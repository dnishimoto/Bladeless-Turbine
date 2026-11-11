//
//  TurbineSceneController.swift
//  Bladeless Turbine
//

import SwiftUI
import Combine
import SceneKit
import UIKit
import simd

final class TurbineSceneController: NSObject, SCNSceneRendererDelegate, ObservableObject {

    @Published var stations: [PressureStation] = []
    @Published var rpm: Float = 0
    @Published var shaftPercent: Float = 0
    @Published var activeCount = 0
    @Published var exhaustedTotal = 0
    @Published var flameTemp: Float = 300
    @Published var fuelFlow: Float = 0
    @Published var fuelUsed: Float = 0
    @Published var thrust: Float = 0
    @Published var exhaustVelocity: Float = 0
    @Published var massFlow: Float = 0
    @Published var pressureRatio: Float = 1
    @Published var tsfc: Float = 0
    @Published var stageCounts: [Int] = Array(repeating: 0, count: EngineStage.allCases.count)
    @Published var turbineTorque: Float = 0
    @Published var compressorLoad: Float = 0
    @Published var starterOn = false
    @Published var cycle = CycleResult()
    @Published var netTorque: Float = 0
    @Published var measuredFlow: Float = 0
    @Published var flightAltitude: Float = 0
    @Published var flightClearance: Float = 0
    @Published var flightSpeed: Float = 0
    @Published var flightMach: Float = 0
    @Published var flightHeading: Float = 0
    @Published var flightPitch: Float = 0
    @Published var flightBank: Float = 0
    @Published var groundAvoid = false
    /// true: camera sits close to the left of the engine (whole engine in view); false: chase view.
    @Published var sideCamera = true

    let cfg: TurbineConfig
    let sim: TurbineCA
    let layout: EngineLayout
    let flight: FlightSim
    let scene = SCNScene()
    let cameraNode = SCNNode()

    private let flightNode = SCNNode()
    private let engineRoot = SCNNode()
    private let terrainNode = SCNNode()
    private var terrainCenter = SIMD2<Float>(repeating: .greatestFiniteMagnitude)
    private var cameraPosition = SIMD3<Float>(0, 0, 0)
    private let turbineSpinNode = SCNNode()
    private let compressorSpinNode = SCNNode()
    private let fanSpinNode = SCNNode()
    private let combustorMaterial = SCNMaterial()

    private var particleNodes: [SCNNode] = []
    private var lastBand: [Int8]
    private var bandGeometries: [SCNGeometry] = []

    private var lastTime: TimeInterval = 0
    private var accumulator: Float = 0
    private var hudTimer: Float = 0

    init(config: TurbineConfig = TurbineConfig()) {
        cfg = config
        sim = TurbineCA(config: config)
        layout = sim.layout
        flight = FlightSim(config: config)
        lastBand = Array(repeating: -1, count: config.maxParticles)
        super.init()
        buildScene()
        stations = sim.stations
    }

    func setFuel(_ v: Float) { sim.fuel = v }
    func setPressureRatio(_ v: Float) { sim.hpcPressureRatio = v }
    func startEngine() { sim.startEngine() }
    func setStick(roll: Float, pitch: Float) { flight.stickRoll = roll; flight.stickPitch = pitch }
    func setRudder(_ v: Float) { flight.rudder = v }
    func toggleCamera() { sideCamera.toggle() }

    func reset() {
        sim.reset()
        flight.reset()
        accumulator = 0
        for n in particleNodes { n.isHidden = true }
        for i in 0..<lastBand.count { lastBand[i] = -1 }
    }

    private func metal(alpha: CGFloat, white: CGFloat = 0.6, tint: UIColor? = nil) -> SCNMaterial {
        let m = SCNMaterial()
        m.lightingModel = .physicallyBased
        m.diffuse.contents = tint ?? UIColor(white: white, alpha: 1)
        m.metalness.contents = 0.9
        m.roughness.contents = 0.4
        m.isDoubleSided = true
        m.transparency = alpha
        return m
    }

    private func buildScene() {
        // Sky, fog and a camera that follows the jet (see updateCamera).
        let sky = UIColor(red: 0.55, green: 0.70, blue: 0.90, alpha: 1)
        scene.background.contents = sky
        scene.fogColor = sky
        scene.fogStartDistance = 2500
        scene.fogEndDistance = 7000

        let cam = SCNCamera()
        cam.zNear = 0.3
        cam.zFar = 9000
        cam.fieldOfView = 50
        cameraNode.camera = cam
        scene.rootNode.addChildNode(cameraNode)

        let ambient = SCNNode()
        ambient.light = SCNLight()
        ambient.light?.type = .ambient
        ambient.light?.intensity = 500
        scene.rootNode.addChildNode(ambient)

        let key = SCNNode()
        key.light = SCNLight()
        key.light?.type = .directional
        key.light?.intensity = 1000
        key.eulerAngles = SCNVector3(-0.6, 0.4, 0)
        scene.rootNode.addChildNode(key)

        scene.rootNode.addChildNode(terrainNode)
        scene.rootNode.addChildNode(flightNode)
        engineRoot.scale = SCNVector3(0.8, 0.8, 0.8)      // ≈ 5 m long
        engineRoot.eulerAngles.y = .pi                    // intake faces the flight direction, exhaust trails
        flightNode.addChildNode(engineRoot)

        buildJetEngineBody()
        buildLabels()
        buildParticles()
    }

    // MARK: Geometry helpers (engine axis = +X)

    /// Tapered shell segment between (x0, r0) and (x1, r1).
    private func frustum(_ x0: Float, _ r0: Float, _ x1: Float, _ r1: Float, material: SCNMaterial) -> SCNNode {
        let cone = SCNCone(topRadius: CGFloat(max(0.0005, r1)), bottomRadius: CGFloat(max(0.0005, r0)),
                           height: CGFloat(max(0.001, x1 - x0)))
        cone.radialSegmentCount = 48
        cone.firstMaterial = material
        let n = SCNNode(geometry: cone)
        n.eulerAngles.z = -.pi / 2          // cone axis +Y → +X
        n.position = SCNVector3((x0 + x1) / 2, 0, 0)
        return n
    }

    private func addShell(_ profile: [SIMD2<Float>], material: SCNMaterial) {
        for i in 0..<(profile.count - 1) {
            let shell = frustum(profile[i].x, profile[i].y, profile[i + 1].x, profile[i + 1].y, material: material)
            if !material.writesToDepthBuffer { shell.renderingOrder = 10 }
            engineRoot.addChildNode(shell)
        }
    }

    private func radialBlades(count: Int, x: Float, rIn: Float, rOut: Float, thickness: Float,
                              chord: Float, twist: Float, material: SCNMaterial, parent: SCNNode) {
        let h = max(0.01, rOut - rIn)
        for k in 0..<count {
            let holder = SCNNode()
            holder.eulerAngles.x = Float(k) * 2 * .pi / Float(count)
            let blade = SCNNode(geometry: SCNBox(width: CGFloat(chord), height: CGFloat(h), length: CGFloat(thickness), chamferRadius: 0))
            blade.geometry?.firstMaterial = material
            blade.position = SCNVector3(x, rIn + h / 2, 0)
            blade.eulerAngles.y = twist
            holder.addChildNode(blade)
            parent.addChildNode(holder)
        }
    }

    private func buildJetEngineBody() {
        let coldMat = metal(alpha: 0.08, white: 0.7, tint: UIColor(red: 0.45, green: 0.65, blue: 0.95, alpha: 1))
        let hotMat = metal(alpha: 0.08, white: 0.7, tint: UIColor(red: 0.95, green: 0.45, blue: 0.30, alpha: 1))
        let drumMat = metal(alpha: 1, white: 0.55)
        let bladeMat = metal(alpha: 1, white: 0.8)
        let statorMat = metal(alpha: 1, white: 0.5)

        combustorMaterial.lightingModel = .blinn
        combustorMaterial.diffuse.contents = UIColor(red: 0.35, green: 0.2, blue: 0.15, alpha: 1)
        combustorMaterial.emission.contents = UIColor(red: 1.0, green: 0.4, blue: 0.08, alpha: 1)
        combustorMaterial.emission.intensity = 0
        combustorMaterial.isDoubleSided = true

        // Casings are see-through glass: tinted, unlit, and they never write depth, so
        // every component behind them stays visible.
        for m in [coldMat, hotMat] {
            m.lightingModel = .constant
            m.writesToDepthBuffer = false
            m.readsFromDepthBuffer = true
            m.transparency = 0.12
        }

        // Casings and rotor drum (tapered compressor, hot section, converging nozzle, tail cone)
        addShell(layout.coldCasingProfile, material: coldMat)
        addShell(layout.hotCasingProfile, material: hotMat)
        addShell(layout.drumProfile, material: drumMat)
        addShell(layout.tailConeProfile, material: drumMat)

        // Intake fan (spins with the fan node)
        engineRoot.addChildNode(fanSpinNode)
        radialBlades(count: 18, x: layout.xLip + 0.05, rIn: 0.10, rOut: 0.55, thickness: 0.012,
                     chord: 0.10, twist: 0.5, material: bladeMat, parent: fanSpinNode)

        // Compressor: 4 rotor rows (spin) interleaved with fixed stator rows
        engineRoot.addChildNode(compressorSpinNode)
        for row in 0..<4 {
            let t = (Float(row) + 0.5) / 4
            let x = layout.lerp(layout.xCompStart, layout.xCompEnd, t)
            let rIn = layout.lerp(0.11, 0.26, t)
            let rOut = layout.lerp(0.52, 0.40, t)
            radialBlades(count: 20 - row, x: x, rIn: rIn, rOut: rOut - 0.02, thickness: 0.008,
                         chord: 0.12, twist: 0.45, material: bladeMat, parent: compressorSpinNode)

            let xs = x + 0.2 * (layout.xCompEnd - layout.xCompStart) / 4
            radialBlades(count: 22 - row, x: xs, rIn: layout.lerp(0.11, 0.26, t + 0.12), rOut: layout.lerp(0.52, 0.40, t + 0.12),
                         thickness: 0.008, chord: 0.10, twist: -0.45, material: statorMat, parent: engineRoot)
        }

        // Annular combustor: glowing flame cans around the shaft
        let cans = 10
        let canX0 = layout.xCompEnd + 0.12, canX1 = layout.xCombEnd + 0.02
        for k in 0..<cans {
            let holder = SCNNode()
            holder.eulerAngles.x = Float(k) * 2 * .pi / Float(cans)
            let cyl = SCNCylinder(radius: 0.075, height: CGFloat(canX1 - canX0))
            cyl.firstMaterial = combustorMaterial
            let can = SCNNode(geometry: cyl)
            can.eulerAngles.z = .pi / 2
            can.position = SCNVector3((canX0 + canX1) / 2, 0.45, 0)
            holder.addChildNode(can)
            engineRoot.addChildNode(holder)
        }

        // Tesla disc turbine: disc normal along +X, spins about the engine axis
        let stackHolder = SCNNode()
        stackHolder.position = SCNVector3(layout.stackCenterX, 0, 0)
        stackHolder.eulerAngles.y = .pi / 2        // local +Z → world +X
        engineRoot.addChildNode(stackHolder)
        stackHolder.addChildNode(turbineSpinNode)

        let diskMat = metal(alpha: 0.9, white: 0.9)
        let solidDisk = Self.diskGeometry(cfg, withHoles: false)
        let drilledDisk = Self.diskGeometry(cfg, withHoles: true)
        solidDisk.materials = [diskMat]
        drilledDisk.materials = [diskMat]
        for i in 0..<cfg.diskCount {
            let node = SCNNode(geometry: i < 3 ? solidDisk : drilledDisk)
            node.scale = SCNVector3(layout.discScale, layout.discScale, 1)
            node.position.z = layout.diskX(i) - layout.stackCenterX
            node.renderingOrder = 5
            turbineSpinNode.addChildNode(node)
        }

        // Main shaft through the drum, discs and tail cone
        let shaftLen = layout.xStackEnd + 0.1 - layout.xNose
        let shaft = SCNCylinder(radius: CGFloat(layout.hubRadius * 0.8), height: CGFloat(shaftLen))
        shaft.firstMaterial = metal(alpha: 1, white: 0.35)
        let shaftNode = SCNNode(geometry: shaft)
        shaftNode.eulerAngles.z = .pi / 2
        shaftNode.position = SCNVector3(layout.xNose + shaftLen / 2, 0, 0)
        engineRoot.addChildNode(shaftNode)
    }

    private func buildLabels() {
        func label(_ text: String, x: Float, y: Float, color: UIColor, size: CGFloat = 0.3) {
            let t = SCNText(string: text, extrusionDepth: 0)
            t.font = UIFont.systemFont(ofSize: size, weight: .bold)
            t.flatness = 0.05
            let m = SCNMaterial()
            m.lightingModel = .constant
            m.diffuse.contents = color
            t.firstMaterial = m
            let n = SCNNode(geometry: t)
            let (mn, mx) = n.boundingBox
            n.pivot = SCNMatrix4MakeTranslation((mn.x + mx.x) / 2, mn.y, 0)
            n.position = SCNVector3(x, y, 0)
            engineRoot.addChildNode(n)
        }
        let top: Float = 0.95
        label("INTAKE", x: (layout.xAirStart + layout.xCompStart) / 2, y: top, color: .systemBlue)
        label("COMPRESSION", x: (layout.xCompStart + layout.xCompEnd) / 2, y: top, color: .systemCyan)
        label("COMBUSTION", x: (layout.xCompEnd + layout.xCombEnd) / 2, y: top, color: .systemYellow)
        label("TESLA TURBINE", x: layout.stackCenterX + 0.2, y: top + 0.15, color: .systemOrange)
        label("EXHAUST", x: (layout.xStackEnd + layout.xExhaustEnd) / 2, y: top, color: .systemPink)
        label("COLD SECTION", x: (layout.xAirStart + layout.xCompEnd) / 2, y: -1.05, color: .systemBlue, size: 0.24)
        label("HOT SECTION", x: (layout.xCompEnd + layout.xExhaustEnd) / 2, y: -1.05, color: .systemRed, size: 0.24)
    }

    private func buildParticles() {
        let colors: [UIColor] = [
            UIColor(red: 1.00, green: 0.95, blue: 0.70, alpha: 1),
            UIColor(red: 1.00, green: 0.60, blue: 0.15, alpha: 1),
            UIColor(red: 0.90, green: 0.20, blue: 0.08, alpha: 1),
            UIColor(red: 0.35, green: 0.55, blue: 0.95, alpha: 1)
        ]
        for c in colors {
            let s = SCNSphere(radius: 0.012)
            s.segmentCount = 6
            let m = SCNMaterial()
            m.lightingModel = .constant
            m.diffuse.contents = c
            m.blendMode = .add
            m.writesToDepthBuffer = false
            s.firstMaterial = m
            bandGeometries.append(s)
        }
        let holder = SCNNode()
        for _ in 0..<cfg.maxParticles {
            let n = SCNNode(geometry: bandGeometries[3])
            n.isHidden = true
            n.renderingOrder = 20
            holder.addChildNode(n)
            particleNodes.append(n)
        }
        engineRoot.addChildNode(holder)
    }

    static func diskGeometry(_ cfg: TurbineConfig, withHoles: Bool) -> SCNShape {
        let path = UIBezierPath()
        let outer = CGFloat(cfg.diskRadius)
        let bore = CGFloat(cfg.hubRadius)

        path.append(UIBezierPath(arcCenter: .zero, radius: outer,
                                 startAngle: 0, endAngle: 2 * .pi, clockwise: true))
        path.append(UIBezierPath(arcCenter: .zero, radius: bore,
                                 startAngle: 2 * .pi, endAngle: 0, clockwise: false))

        if withHoles {
            let rIn = CGFloat(cfg.holeInnerRadius)
            let rOut = CGFloat(cfg.holeOuterRadius)
            let half = CGFloat(cfg.holeHalfAngle)
            for k in 0..<cfg.holeCount {
                let c = CGFloat(k) * 2 * .pi / CGFloat(cfg.holeCount)
                let a0 = c - half, a1 = c + half
                let hole = UIBezierPath()
                hole.addArc(withCenter: .zero, radius: rOut,
                            startAngle: a1, endAngle: a0, clockwise: false)
                hole.addArc(withCenter: .zero, radius: rIn,
                            startAngle: a0, endAngle: a1, clockwise: true)
                hole.close()
                path.append(hole)
            }
        }
        path.usesEvenOddFillRule = true
        path.flatness = 0.002

        let shape = SCNShape(path: path, extrusionDepth: CGFloat(cfg.diskThickness))
        shape.chamferRadius = 0
        return shape
    }

    private func updateTerrain() {
        let p = SIMD2<Float>(flight.position.x, flight.position.z)
        if simd_distance(p, terrainCenter) > 1800 {
            terrainCenter = p
            terrainNode.geometry = Terrain.mesh(centerX: p.x, centerZ: p.y)
        }
    }

    /// Closed camera spline around the engine, in jet-local metres (x forward, y up, z right).
    private static let orbitPoints: [SIMD3<Float>] = [
        SIMD3( 0.4, 0.3, -3.6),   // left, mid-engine
        SIMD3( 2.6, 0.9, -2.8),   // left, at the intake
        SIMD3( 4.0, 0.5, -0.6),   // in front of the intake
        SIMD3( 3.0, 1.7,  2.2),   // front right, above
        SIMD3( 0.4, 0.7,  3.6),   // right, mid-engine
        SIMD3(-2.2, 0.4,  2.9),   // right, at the Tesla discs / nozzle
        SIMD3(-3.8, 0.9,  0.4),   // behind the exhaust
        SIMD3(-2.5, 1.8, -2.3)    // left rear, above
    ]
    private var orbitT: Float = 0

    private func catmullRom(_ pts: [SIMD3<Float>], _ t: Float) -> SIMD3<Float> {
        let n = pts.count
        let u = t - floor(t)
        let f = u * Float(n)
        let i = Int(f) % n
        let x = f - Float(Int(f))
        let p0 = pts[(i + n - 1) % n], p1 = pts[i], p2 = pts[(i + 1) % n], p3 = pts[(i + 2) % n]
        let x2 = x * x, x3 = x2 * x
        let c1: SIMD3<Float> = (p2 - p0) * x
        let c2: SIMD3<Float> = (p0 * 2 - p1 * 5 + p2 * 4 - p3) * x2
        let c3: SIMD3<Float> = (p1 * 3 - p0 - p2 * 3 + p3) * x3
        let sum: SIMD3<Float> = p1 * 2 + c1 + c2 + c3
        return sum * 0.5
    }

    private func updateCamera(dt: Float) {
        let f = flight.forward, pos = flight.position
        let worldUp = SIMD3<Float>(0, 1, 0)
        cameraNode.camera?.projectionDirection = .vertical
        cameraNode.camera?.fieldOfView = sideCamera ? 60 : 50

        if sideCamera {
            // Orbit: follow the spline around the engine, always looking at the engine axis.
            orbitT += dt / 30                                   // one lap every 30 s
            let local = catmullRom(Self.orbitPoints, orbitT)
            let q = flight.orientation
            let camPos = pos + q.act(local)
            let focusX = max(-1.8, min(2.2, local.x * 0.6)) + 0.4   // look along the engine as we pass
            let focus = pos + q.act(SIMD3(focusX, 0, 0))
            let up = q.act(worldUp)
            cameraPosition = camPos
            cameraNode.simdPosition = camPos
            cameraNode.look(at: SCNVector3(focus.x, focus.y, focus.z), up: SCNVector3(up.x, up.y, up.z),
                            localFront: SCNVector3(0, 0, -1))
            return
        }

        let target = pos - f * 15 + worldUp * 4.5
        let look = pos + f * 8
        let k = 1 - exp(-8 * dt)
        cameraPosition = (cameraPosition == .zero) ? target : cameraPosition + (target - cameraPosition) * k
        let minY = Terrain.height(cameraPosition.x, cameraPosition.z) + 3
        cameraPosition.y = max(cameraPosition.y, minY)
        cameraNode.simdPosition = cameraPosition
        cameraNode.look(at: SCNVector3(look.x, look.y, look.z))
    }

    private func band(for temp: Float) -> Int8 {
        temp > 1200 ? 0 : temp > 900 ? 1 : temp > 600 ? 2 : 3
    }

    func renderer(_ renderer: SCNSceneRenderer, updateAtTime time: TimeInterval) {
        if lastTime == 0 { lastTime = time }
        let frame = min(Float(time - lastTime), 0.1)
        lastTime = time

        // Flight: the engine's mass flow sets the speed; altitude and speed feed back into the cycle.
        sim.altitude = flight.altitude
        sim.airspeed = flight.speed
        flight.step(dt: frame, cycle: sim.cycle)
        flightNode.simdPosition = flight.position
        flightNode.simdOrientation = flight.orientation
        updateTerrain()
        updateCamera(dt: frame)

        accumulator += frame
        let tickDt = 1 / cfg.tickRate
        while accumulator >= tickDt {
            sim.tick()
            accumulator -= tickDt
        }
        let alpha = accumulator / tickDt

        let angle = sim.diskAngle + sim.omega * accumulator
        turbineSpinNode.eulerAngles.z = angle     // disc node: local Z is the engine axis
        compressorSpinNode.eulerAngles.x = angle
        fanSpinNode.eulerAngles.x = angle

        combustorMaterial.emission.intensity = CGFloat(max(0, min(1, (sim.flameTemp - 300) / 1500)))

        let particles = sim.particles
        for i in 0..<particles.count {
            let p = particles[i]
            let node = particleNodes[i]
            if !p.active {
                if !node.isHidden { node.isHidden = true }
                continue
            }
            node.isHidden = false
            let q = sim.renderPosition(of: p, alpha: alpha)
            node.position = SCNVector3(q.x, q.y, q.z)
            let s = sim.visualScale(of: p)
            node.scale = SCNVector3(s, s, s)
            let b = band(for: p.temp)
            if b != lastBand[i] {
                node.geometry = bandGeometries[Int(b)]
                lastBand[i] = b
            }
        }

        hudTimer += frame
        if hudTimer > 0.12 {
            hudTimer = 0
            let snapshot = sim.stations
            let rpm = sim.rpm, pct = sim.shaftFraction * 100
            let active = sim.activeCount, out = sim.exhaustedTotal
            let flame = sim.flameTemp, flow = sim.fuelFlowGramsPerSec, used = sim.fuelUsedGrams
            let thrust = sim.thrust, tq = sim.turbineTorque, ld = sim.compressorLoad
            let starter = sim.starterOn
            let cyc = sim.cycle
            let measured = sim.measuredFlow
            let fs = flight
            let fAlt = fs.altitude, fClr = fs.clearance, fSpd = fs.speed, fMach = fs.mach
            let fHdg = fs.heading, fPit = fs.pitchAngle * 180 / .pi, fBank = fs.bank * 180 / .pi, fGCAS = fs.groundAvoid
            let net = cyc.turbineTorque - cyc.compressorTorque - cyc.frictionTorque
            let vExit = sim.exhaustVelocity, mdot = sim.exhaustMassFlow
            let opr = sim.overallPressureRatio, sfc = sim.tsfc, counts = sim.stageCounts
            DispatchQueue.main.async {
                self.stations = snapshot
                self.rpm = rpm
                self.shaftPercent = pct
                self.activeCount = active
                self.exhaustedTotal = out
                self.flameTemp = flame
                self.fuelFlow = flow
                self.fuelUsed = used
                self.thrust = thrust
                self.exhaustVelocity = vExit
                self.massFlow = mdot
                self.pressureRatio = opr
                self.tsfc = sfc
                self.stageCounts = counts
                self.turbineTorque = tq
                self.compressorLoad = ld
                self.starterOn = starter
                self.cycle = cyc
                self.measuredFlow = measured
                self.flightAltitude = fAlt
                self.flightClearance = fClr
                self.flightSpeed = fSpd
                self.flightMach = fMach
                self.flightHeading = fHdg
                self.flightPitch = fPit
                self.flightBank = fBank
                self.groundAvoid = fGCAS
                self.netTorque = net
            }
        }
    }
}

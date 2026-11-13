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
    private static let ringSpacing: Float = 300.0
    private let wingNode = SCNNode()
    private let wingMaterial = SCNMaterial()
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
    @Published var recoveryActive = false
    @Published var canyonReturn = false
    @Published var score: Int = 0
    @Published var orbitPercent: Int = 0
    /// true: camera sits close to the left of the engine (whole engine in view); false: chase view.
    @Published var sideCamera = false {
        didSet {
            // Every switch restarts the lap from the start of the ring.
            orbitT = 0
            orbitHeight = 0
            orbitFinishing = false
            print("[camera] sideCamera =", sideCamera)
        }
    }

    let cfg: TurbineConfig
    let sim: TurbineCA
    let layout: EngineLayout
    let flight: FlightSim
    let scene = SCNScene()
    let cameraNode = SCNNode()
    /// Camera rig: a child of `flightNode` (the engine's parent) sitting at the engine centre.
    /// The camera is a child of the rig, so rotating the rig's local rotation swings the camera
    /// around the engine, and everything inherits the jet's own rotation through `flightNode`.
    private let cameraRig = SCNNode()

    private let flightNode = SCNNode()
    private let engineRoot = SCNNode()
    private let terrainNode = SCNNode()
    private let splineNode = SCNNode()
    private var splineAnchorX: Float = .greatestFiniteMagnitude
    private let ringsRootNode = SCNNode()
    private struct CanyonRing {
        let index: Int
        let center: SIMD3<Float>
        let normal: SIMD3<Float>
        let radius: Float
        let node: SCNNode
        var scored: Bool = false
    }
    private var canyonRings: [Int: CanyonRing] = [:]
    private var previousFlightPosition: SIMD3<Float>?
    private static let ringRadius: Float = 38.0
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
    /// The button just flips `sideCamera`; the renderer reads it directly, so setting
    /// `sideCamera` from anywhere switches between chase and the 360° engine orbit.
    func toggleCamera() {
        if Thread.isMainThread { sideCamera.toggle() }
        else { DispatchQueue.main.async { [weak self] in self?.sideCamera.toggle() } }
    }

    func reset() {
        sim.reset()
        flight.reset()
        accumulator = 0
        previousFlightPosition = nil
        for ring in canyonRings.values {
            ring.node.removeFromParentNode()
        }
        canyonRings.removeAll()
        DispatchQueue.main.async { [weak self] in
            self?.score = 0
        }
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
        scene.fogStartDistance = 7000
        scene.fogEndDistance = 15000

        flightNode.simdPosition = flight.position
        flightNode.simdOrientation = flight.orientation

        scene.rootNode.addChildNode(flightNode)
        
        let cam = SCNCamera()
        cam.zNear = 0.1
        cam.zFar = 22000
        cam.fieldOfView = 50
        cameraNode.camera = cam
        // The camera is a child of the jet (the engine's parent), so it lives in the jet's local
        // frame: it can never lag behind the jet, whatever the speed.
        cameraRig.simdPosition = engineCentreLocal
        flightNode.addChildNode(cameraRig)
        cameraRig.addChildNode(cameraNode)

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
        scene.rootNode.addChildNode(splineNode)
        scene.rootNode.addChildNode(ringsRootNode)
        scene.rootNode.addChildNode(flightNode)
        engineRoot.scale = SCNVector3(0.8, 0.8, 0.8)      // ≈ 5 m long
        engineRoot.eulerAngles.y = .pi                    // intake faces the flight direction, exhaust trails
        flightNode.addChildNode(engineRoot)

        buildJetEngineBody()
        buildLabels()
        buildParticles()
        buildTransparentWings()
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

    // MARK: Canyon guide splines (centre line, left/right boundaries, variable flight path)

    private func lineNode(_ points: [SCNVector3], color: UIColor) -> SCNNode {
        var indices = [Int32]()
        for i in 0..<(points.count - 1) { indices.append(Int32(i)); indices.append(Int32(i + 1)) }
        let geo = SCNGeometry(sources: [SCNGeometrySource(vertices: points)],
                              elements: [SCNGeometryElement(indices: indices, primitiveType: .line)])
        let m = SCNMaterial()
        m.lightingModel = .constant
        m.diffuse.contents = color
        m.emission.contents = color
        m.isDoubleSided = true
        geo.materials = [m]
        return SCNNode(geometry: geo)
    }

    private func updateSplines() {
        let x0 = flight.position.x
        guard abs(x0 - splineAnchorX) > 400 else { return }
        splineAnchorX = x0

        let step: Float = 80
        let start = (x0 / step).rounded() * step - 800
        var centre = [SCNVector3](), left = [SCNVector3](), right = [SCNVector3](), guide = [SCNVector3]()
        for i in 0..<180 {
            let x = start + Float(i) * step
            func lift(_ p: SIMD3<Float>) -> SCNVector3 {
                SCNVector3(p.x, Terrain.height(p.x, p.z) + 12, p.z)
            }
            centre.append(lift(Terrain.canyonCenterPath(x)))
            left.append(lift(Terrain.canyonLeftBoundary(x)))
            right.append(lift(Terrain.canyonRightBoundary(x)))
            guide.append(lift(Terrain.canyonVariablePath(x)))
        }
        splineNode.childNodes.forEach { $0.removeFromParentNode() }
        splineNode.addChildNode(lineNode(centre, color: .systemCyan))      // central spline
        splineNode.addChildNode(lineNode(left, color: .systemGreen))       // guide boundary, left
        splineNode.addChildNode(lineNode(right, color: .systemGreen))      // guide boundary, right
        splineNode.addChildNode(lineNode(guide, color: .systemYellow))     // variable flight path
    }

    // MARK: Canyon Scoring Rings

    private func makeScoringRingNode(center: SIMD3<Float>, normal: SIMD3<Float>, radius: Float) -> SCNNode {
        let torus = SCNTorus(ringRadius: CGFloat(radius), pipeRadius: 2.2)
        torus.ringSegmentCount = 48
        torus.pipeSegmentCount = 12

        let mat = SCNMaterial()
        mat.lightingModel = .constant
        mat.diffuse.contents = UIColor.systemYellow
        mat.emission.contents = UIColor.systemOrange
        mat.transparency = 0.88
        torus.firstMaterial = mat

        let node = SCNNode(geometry: torus)
        node.simdPosition = center
        // SCNTorus lies in the XZ plane with its hole axis along +Y; rotate +Y to align with the canyon tangent.
        node.simdOrientation = simd_quatf(from: SIMD3<Float>(0, 1, 0), to: simd_normalize(normal))
        return node
    }

    private func updateScoringRings(
            prevPos: SIMD3<Float>,
            currPos: SIMD3<Float>
        ) {
            let spacing = Self.ringSpacing
            let currentIndex = Int(floor(currPos.x / spacing))

            // Maintain rings in both directions so turning around still works.
            let ringsEachDirection = 24
            let minimumIndex = currentIndex - ringsEachDirection
            let maximumIndex = currentIndex + ringsEachDirection

            func ringCenter(at x: Float) -> SIMD3<Float> {
                let path = Terrain.canyonVariablePath(x)
                let floorY = Terrain.height(path.x, path.z)

                let rimY: Float = 1240.0
                let centerY = floorY + (rimY - floorY) * 0.5

                return SIMD3<Float>(path.x, centerY, path.z)
            }

            // 1. Continuously fill missing ring positions.
            for index in minimumIndex...maximumIndex {
                guard canyonRings[index] == nil else { continue }

                let x = Float(index) * spacing
                let center = ringCenter(at: x)

                let before = ringCenter(at: x - 25.0)
                let after = ringCenter(at: x + 25.0)
                let direction = after - before

                let normal: SIMD3<Float>
                if simd_length_squared(direction) > 0.0001 {
                    normal = simd_normalize(direction)
                } else {
                    normal = SIMD3<Float>(1, 0, 0)
                }

                let node = makeScoringRingNode(
                    center: center,
                    normal: normal,
                    radius: Self.ringRadius
                )

                ringsRootNode.addChildNode(node)

                canyonRings[index] = CanyonRing(
                    index: index,
                    center: center,
                    normal: normal,
                    radius: Self.ringRadius,
                    node: node
                )
            }

            // 2. Check if the engine passes through any active ring.
            for (index, ring) in canyonRings where !ring.scored {
                // Vector from ring center to previous and current positions
                let vPrev = prevPos - ring.center
                let vCurr = currPos - ring.center
                
                let dPrev = simd_dot(vPrev, ring.normal)
                let dCurr = simd_dot(vCurr, ring.normal)

                // Check if the aircraft crossed the ring plane (sign change of dot product)
                if dPrev * dCurr <= 0 {
                    let intersectionPoint = currPos // Approximate intersection point
                    let distanceFromCenter = simd_distance(intersectionPoint, ring.center)

                    // Verify if the engine passed through the ring's opening
                    if distanceFromCenter <= ring.radius {
                        canyonRings[index]?.scored = true
                        
                        // Trigger particle burst at the ring center
                        spawnRingBurst(at: ring.center)
                        
                        // Increment score on the main thread
                        DispatchQueue.main.async { [weak self] in
                            self?.score += 100
                        }
                    }
                }
            }

            // 3. Remove distant rings in either direction.
            let expiredIndices = canyonRings.keys.filter {
                $0 < minimumIndex || $0 > maximumIndex
            }

            for index in expiredIndices {
                canyonRings[index]?.node.removeFromParentNode()
                canyonRings.removeValue(forKey: index)
            }
        }

    private func spawnRingBurst(at position: SIMD3<Float>) {
        let burstNode = SCNNode()
        burstNode.simdPosition = position

        let particleSystem = SCNParticleSystem()
        particleSystem.particleColor = UIColor.systemGreen
        particleSystem.particleSize = 0.8
        particleSystem.birthRate = 350
        particleSystem.emissionDuration = 0.08
        particleSystem.loops = false

        particleSystem.particleLifeSpan = 0.6
        particleSystem.particleVelocity = 45.0
        particleSystem.particleVelocityVariation = 15.0
        particleSystem.spreadingAngle = 180.0
        particleSystem.blendMode = .additive

        // Uses SceneKit's default particle appearance.
        // Set particleImage here if you have a custom texture.

        burstNode.addParticleSystem(particleSystem)
        scene.rootNode.addChildNode(burstNode)

        let cleanupDelay = particleSystem.emissionDuration
            + TimeInterval(particleSystem.particleLifeSpan)
            + 0.1

        burstNode.runAction(.sequence([
            .wait(duration: cleanupDelay),
            .removeFromParentNode()
        ]))
    }

    // MARK: Streaming terrain

    private struct TileKey: Hashable { let x: Int, z: Int }
    private var tiles: [TileKey: (node: SCNNode, cells: Int)] = [:]
    private var pendingTiles: [TileKey: Int] = [:]
    private var readyTiles: [(key: TileKey, cells: Int, data: Terrain.TileData)] = []
    private let readyLock = NSLock()
    private let tileQueue = DispatchQueue(label: "terrain.tiles", qos: .userInitiated, attributes: .concurrent)
    /// Tiles kept around the jet: radius 6 → 13 × 13 tiles of 1.92 km, about 11.5 km each way.
    private static let tileRadius = 6

    private static func tileDetail(_ ring: Int) -> Int { ring <= 1 ? 32 : (ring <= 3 ? 16 : 8) }

    private func updateTerrain() {
        let size = Terrain.tileSize
        let cx = Int(floor(flight.position.x / size)), cz = Int(floor(flight.position.z / size))
        let radius = Self.tileRadius

        // 1. Add finished tiles (at most two per frame to avoid hitches).
        readyLock.lock()
        let batch = Array(readyTiles.prefix(2))
        readyTiles.removeFirst(min(2, readyTiles.count))
        readyLock.unlock()
        for item in batch {
            if pendingTiles[item.key] == item.cells { pendingTiles[item.key] = nil }
            guard abs(item.key.x - cx) <= radius + 1, abs(item.key.z - cz) <= radius + 1 else { continue }
            tiles[item.key]?.node.removeFromParentNode()
            let node = Terrain.tileNode(from: item.data, name: "tile_\(item.key.x)_\(item.key.z)")
            terrainNode.addChildNode(node)
            tiles[item.key] = (node, item.cells)
        }

        // 2. Drop tiles that are now too far away.
        for (key, tile) in tiles where abs(key.x - cx) > radius + 1 || abs(key.z - cz) > radius + 1 {
            tile.node.removeFromParentNode()
            tiles[key] = nil
        }

        // 3. Request missing or wrong-detail tiles, nearest first.
        var wanted: [(key: TileKey, ring: Int)] = []
        for dz in -radius...radius {
            for dx in -radius...radius {
                wanted.append((TileKey(x: cx + dx, z: cz + dz), max(abs(dx), abs(dz))))
            }
        }
        wanted.sort { $0.ring < $1.ring }
        for item in wanted {
            if pendingTiles.count >= 6 { break }
            let cells = Self.tileDetail(item.ring)
            if tiles[item.key]?.cells == cells || pendingTiles[item.key] == cells { continue }
            pendingTiles[item.key] = cells
            let key = item.key
            tileQueue.async { [weak self] in
                let data = Terrain.tileData(tileX: key.x, tileZ: key.z, cellsPerSide: cells)
                guard let self else { return }
                self.readyLock.lock()
                self.readyTiles.append((key, cells, data))
                self.readyLock.unlock()
            }
        }
    }

    // MARK: Camera
    //
    // Hierarchy:  flightNode (jet position + rotation)
    //               ├─ engineRoot (the engine)
    //               └─ cameraRig (at the engine centre; its LOCAL rotation orbits the camera)
    //                    └─ cameraNode (offset from the engine, looks at the engine)

    /// Engine centre in jet-local coordinates (engineRoot is scaled 0.8 and yawed 180°).
    private var engineCentreLocal: SIMD3<Float> {
        let midX = (layout.xNose + layout.xExhaustEnd) * 0.5
        return SIMD3<Float>(-midX * Float(engineRoot.scale.x), 0, 0)
    }

    /// Home pose, jet-local: behind and above the engine, looking ahead of the nose.
    private static let homeLocal = SIMD3<Float>(-15, 4.5, 0)
    private static let homeLookLocal = SIMD3<Float>(8, 0, 0)

    private func updateCamera(dt: Float) {
        cameraNode.camera?.projectionDirection = .vertical
        cameraNode.camera?.fieldOfView = sideCamera ? 60 : 50

        let centre = engineCentreLocal
        // Home pose relative to the rig (the engine centre).
        var offset = Self.homeLocal - centre
        var look = Self.homeLookLocal - centre
        var rigRotation = simd_quatf(angle: 0, axis: SIMD3<Float>(0, 1, 0))

        if sideCamera {
            let pose = orbitPose(dt: dt)
            offset = pose.offset
            look = pose.look
            rigRotation = pose.rotation
        }

        // Keep the camera out of the ground: world position = jet + jet rotation · (rig + rotation · offset).
        let world = flight.position + flight.orientation.act(centre + rigRotation.act(offset))
        let floorY = Terrain.height(world.x, world.z) + 3
        if world.y < floorY { offset.y += floorY - world.y }

        cameraRig.simdPosition = centre                     // rig sits at the engine centre
        cameraRig.simdOrientation = rigRotation             // local rotation: swings the camera round
        cameraNode.simdPosition = offset                    // offset from the engine, rig space

        // SCNNode.look(at:up:localFront:) takes WORLD-space coordinates, even for a child node, so
        // convert the rig-space look-at point to world space through the rig and the jet.
        let q = flight.orientation
        let lookWorld = flight.position + q.act(centre + rigRotation.act(look))
        let upWorld = q.act(SIMD3<Float>(0, 1, 0))          // the jet's up: the view banks with the jet
        cameraNode.look(
            at: SCNVector3(lookWorld.x, lookWorld.y, lookWorld.z),
            up: SCNVector3(upWorld.x, upWorld.y, upWorld.z),
            localFront: SCNVector3(0, 0, -1)
        )
    }

    // MARK: Orbit loop

    /// Progress through the loop, 0…1. 0 and 1 are both the home pose.
    private var orbitT: Float = 0
    private var orbitHeight: Float = 0
    private var orbitFinishing = false
    /// Seconds for the whole loop (leave home, circle the engine, return home).
    private static let orbitLapSeconds: Float = 16

    /// Closed spline of the camera's offset from the engine in the rig's own (unrotated) frame, on
    /// the engine's rear side (x = −distance, y = height): home → swoop in → close pass with a small
    /// height wave → swoop out → home. Rotating the rig about the engine's vertical axis sweeps
    /// this curve 360° around the engine.
    private var meridianSpline: [SIMD3<Float>] {
        let home = Self.homeLocal - engineCentreLocal
        return [
            home,
            SIMD3<Float>(-9, 3.2, 0),
            SIMD3<Float>(-4.4, 1.6, 0),
            SIMD3<Float>(-3.2, 0.5, 0),
            SIMD3<Float>(-3.0, 0.9, 0),
            SIMD3<Float>(-3.2, 0.5, 0),
            SIMD3<Float>(-4.4, 1.6, 0),
            SIMD3<Float>(-9, 3.2, 0)
        ]
    }

    /// Closed Catmull-Rom spline through `pts`; t in 0…1 wraps around the loop.
    private static func closedSpline(_ pts: [SIMD3<Float>], _ t: Float) -> SIMD3<Float> {
        let n = pts.count
        let u = (t - floor(t)) * Float(n)
        let i = min(n - 1, Int(u))
        let x = u - Float(i)
        let p0 = pts[(i + n - 1) % n], p1 = pts[i], p2 = pts[(i + 1) % n], p3 = pts[(i + 2) % n]
        let a: SIMD3<Float> = p1 * 2
        let b: SIMD3<Float> = (p2 - p0) * x
        let c: SIMD3<Float> = (p0 * 2 - p1 * 5 + p2 * 4 - p3) * (x * x)
        let d: SIMD3<Float> = (p1 * 3 - p0 - p2 * 3 + p3) * (x * x * x)
        return (a + b + c + d) * 0.5
    }

    /// Drag while orbiting: horizontal scrubs along the loop, vertical raises or lowers the camera.
    func orbitDrag(dx: Float, dy: Float) {
        guard sideCamera else { return }
        orbitT = max(0, min(0.999, orbitT + dx * 0.002))
        orbitHeight = max(-1.2, min(1.8, orbitHeight - dy * 0.01))
    }

    /// Advances the loop. Returns the camera's offset from the engine and its look-at point (both in
    /// rig space) plus the rig's local rotation. At the end of the loop the camera is back at the
    /// home pose and `sideCamera` switches off.
    private func orbitPose(dt: Float) -> (offset: SIMD3<Float>, look: SIMD3<Float>, rotation: simd_quatf) {
        orbitT = min(1, orbitT + max(0, dt) / Self.orbitLapSeconds)

        var offset = Self.closedSpline(meridianSpline, orbitT)
        offset.y += orbitHeight * sin(.pi * orbitT)         // drag height fades out toward home

        // Rig rotation: one full 360° turn about the engine's vertical axis, eased in the middle
        // of the loop; 0 at both ends so the camera lands exactly on the home pose.
        let s = max(0, min(1, (orbitT - 0.15) / 0.70))
        let ease = s * s * (3 - 2 * s)
        let rotation = simd_quatf(angle: 2 * .pi * ease, axis: SIMD3<Float>(0, 1, 0))

        // Look at the engine centre while close; ease to the home look-at at the ends.
        let home = max(0, min(1, (orbitT - 0.85) / 0.15)) + max(0, min(1, (0.15 - orbitT) / 0.15))
        let homeLook = Self.homeLookLocal - engineCentreLocal
        let look = homeLook * min(1, home)

        let pct = Int(orbitT * 100)
        if pct != orbitPercent { DispatchQueue.main.async { self.orbitPercent = pct } }

        if orbitT >= 1 && !orbitFinishing {
            orbitFinishing = true
            DispatchQueue.main.async { [weak self] in self?.sideCamera = false }   // back to home
        }
        return (offset, look, rotation)
    }

    private func band(for temp: Float) -> Int8 {
        temp > 1200 ? 0 : temp > 900 ? 1 : temp > 600 ? 2 : 3
    }

    func renderer(
        _ renderer: SCNSceneRenderer,
        updateAtTime time: TimeInterval
    ) {
        // MARK: - Frame Timing

        if lastTime == 0 {
            lastTime = time
            return
        }

        let elapsed = Float(time - lastTime)
        lastTime = time

        let frame = min(max(elapsed, 0), 0.1)
        guard frame > 0 else { return }

        // MARK: - Engine / CA Simulation

        // Feed current flight conditions into the engine.
        sim.altitude = flight.altitude
        sim.airspeed = flight.speed

        // Advance the engine CA at its configured fixed timestep.
        accumulator += frame

        let tickDt = 1 / cfg.tickRate

        while accumulator >= tickDt {
            sim.tick()
            accumulator -= tickDt
        }

        let alpha = accumulator / tickDt

        // MARK: - Aircraft Flight Physics

        let prevPos = previousFlightPosition ?? flight.position

        // FlightSim receives the latest engine cycle.
        flight.step(
            dt: frame,
            cycle: sim.cycle
        )

        previousFlightPosition = flight.position

        // MARK: - Engine / Aircraft Transform

        // The engine is a child of flightNode and inherits its position and orientation.
        flightNode.simdPosition = flight.position
        flightNode.simdOrientation = flight.orientation

        // MARK: - Terrain, Flight Path, and Scoring Rings

        updateTerrain()
        updateSplines()
        updateScoringRings(prevPos: prevPos, currPos: flight.position)

        // MARK: - Camera

        updateCamera(dt: frame)

        // MARK: - Rotor Animation

        let angle = sim.diskAngle + sim.omega * accumulator

        turbineSpinNode.eulerAngles.z = angle
        compressorSpinNode.eulerAngles.x = angle
        fanSpinNode.eulerAngles.x = angle

        // MARK: - Combustor

        combustorMaterial.emission.intensity = CGFloat(
            max(0, min(1, (sim.flameTemp - 300) / 1500))
        )

        // MARK: - Particle Rendering

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

        // MARK: - HUD Updates

        hudTimer += frame

        if hudTimer > 0.12 {
            hudTimer = 0

            let snapshot = sim.stations

            let rpm = sim.rpm
            let pct = sim.shaftFraction * 100
            let active = sim.activeCount
            let out = sim.exhaustedTotal

            let flame = sim.flameTemp
            let flow = sim.fuelFlowGramsPerSec
            let used = sim.fuelUsedGrams

            let thrust = sim.thrust
            let tq = sim.turbineTorque
            let ld = sim.compressorLoad

            let starter = sim.starterOn
            let cyc = sim.cycle
            let measured = sim.measuredFlow

            let fs = flight

            let fAlt = fs.altitude
            let fClr = fs.clearance
            let fSpd = fs.speed
            let fMach = fs.mach

            let fHdg = fs.heading
            let fPit = fs.pitchAngle * 180 / .pi
            let fBank = fs.bank * 180 / .pi
            let fGCAS = fs.groundAvoid
            let fRecovery = fs.recoveryActive
            let fCanyon = fs.canyonReturnActive

            let net = cyc.turbineTorque
                - cyc.compressorTorque
                - cyc.frictionTorque

            let vExit = sim.exhaustVelocity
            let mdot = sim.exhaustMassFlow

            let opr = sim.overallPressureRatio
            let sfc = sim.tsfc
            let counts = sim.stageCounts

            DispatchQueue.main.async { [weak self] in
                guard let self else { return }

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
                self.recoveryActive = fRecovery
                self.canyonReturn = fCanyon
                self.netTorque = net
            }
        }
    }
    // MARK: - F-16 Style Transparent Wings

    private func buildTransparentWings() {

        wingNode.name = "F16TransparentWings"

        wingMaterial.lightingModel = .constant
        wingMaterial.diffuse.contents = UIColor(
            red: 0.15,
            green: 0.72,
            blue: 1.0,
            alpha: 1.0
        )
        wingMaterial.transparency = 0.24
        wingMaterial.isDoubleSided = true
        wingMaterial.writesToDepthBuffer = false
        wingMaterial.readsFromDepthBuffer = true

        let semiSpan: CGFloat = 4.8
        let rootLeading: CGFloat = 1.4
        let rootTrailing: CGFloat = -2.5
        let tipLeading: CGFloat = -0.9
        let tipTrailing: CGFloat = -1.9

        for side: CGFloat in [-1, 1] {

            let path = UIBezierPath()

            // F-16 swept trapezoidal wing outline.
            path.move(to: CGPoint(x: rootLeading, y: 0))

            path.addLine(to: CGPoint(
                x: tipLeading,
                y: side * semiSpan
            ))

            path.addLine(to: CGPoint(
                x: tipTrailing,
                y: side * semiSpan
            ))

            path.addLine(to: CGPoint(
                x: rootTrailing,
                y: 0
            ))

            path.close()

            let geometry = SCNShape(
                path: path,
                extrusionDepth: 0.035
            )

            geometry.chamferRadius = 0
            geometry.firstMaterial = wingMaterial

            let wing = SCNNode(geometry: geometry)

            wing.name = side < 0
                ? "LeftF16Wing"
                : "RightF16Wing"

            // Convert the shape's XY plane into aircraft XZ.
            wing.eulerAngles.x = .pi / 2

            wing.position = SCNVector3(0, 0.22, 0)

            wingNode.addChildNode(wing)

            // Leading-edge highlight.
            let edge = SCNGeometry(
                sources: [
                    SCNGeometrySource(vertices: [
                        SCNVector3(
                            Float(rootLeading), 0.025, 0
                        ),
                        SCNVector3(
                            Float(tipLeading), 0.025,
                            Float(side * semiSpan)
                        )
                    ])
                ],
                elements: [
                    SCNGeometryElement(
                        indices: [0, 1],
                        primitiveType: .line
                    )
                ]
            )

            let edgeMaterial = SCNMaterial()
            edgeMaterial.lightingModel = .constant
            edgeMaterial.diffuse.contents = UIColor.cyan
            edgeMaterial.emission.contents = UIColor.cyan
            edgeMaterial.transparency = 0.8
            edge.materials = [edgeMaterial]

            let edgeNode = SCNNode(geometry: edge)
            wingNode.addChildNode(edgeNode)
        }

        flightNode.addChildNode(wingNode)
    }
}

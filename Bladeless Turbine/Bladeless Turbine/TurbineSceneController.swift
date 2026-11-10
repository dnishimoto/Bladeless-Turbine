//
//  TurbineSceneController.swift
//  Bladeless Turbine
//
//  Builds the SceneKit scene for the whole engine and drives the CA each frame:
//
//    air inlet + fan → bladed compressor → combustor (fuel injectors) →
//    tangential inlet manifold → Tesla disc turbine (three openings per disc) →
//    exhaust collector → rear nozzle,  with a drive shaft from the Tesla rotor
//    back to the compressor.
//
//  Gas particles are drawn from the CA state; the colour runs from cold blue
//  (intake air) through red/orange to white-hot (combustor/turbine inlet), and
//  hot gas is drawn larger to show expansion.
//

//
//  TurbineSceneController.swift
//  Bladeless Turbine
//

import SwiftUI
import Combine
import SceneKit
import UIKit

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

    let cfg: TurbineConfig
    let sim: TurbineCA
    let scene = SCNScene()
    let cameraNode = SCNNode()

    private let diskSpin = SCNNode()
    private let compressorSpin = SCNNode()
    private let fanSpin = SCNNode()
    private let combustorMaterial = SCNMaterial()
    private let injectorMaterial = SCNMaterial()

    private var particleNodes: [SCNNode] = []
    private var lastBand: [Int8]
    private var bandGeometries: [SCNGeometry] = []

    private var lastTime: TimeInterval = 0
    private var accumulator: Float = 0
    private var hudTimer: Float = 0

    init(config: TurbineConfig = TurbineConfig()) {
        cfg = config
        sim = TurbineCA(config: config)
        lastBand = Array(repeating: -1, count: config.maxParticles)
        super.init()
        buildScene()
        stations = sim.stations
    }

    func setFuel(_ v: Float) { sim.fuel = v }
    func setPressureRatio(_ v: Float) { sim.designPressureRatio = v }
    func startEngine() { sim.startEngine() }

    func reset() {
        sim.reset()
        accumulator = 0
        for n in particleNodes { n.isHidden = true }
        for i in 0..<lastBand.count { lastBand[i] = -1 }
    }

    private func metal(alpha: CGFloat, white: CGFloat = 0.6) -> SCNMaterial {
        let m = SCNMaterial()
        m.lightingModel = .physicallyBased
        m.diffuse.contents = UIColor(white: white, alpha: 1)
        m.metalness.contents = 0.9
        m.roughness.contents = 0.4
        m.isDoubleSided = true
        m.transparency = alpha
        return m
    }

    private func buildScene() {
        scene.background.contents = UIColor(red: 0.02, green: 0.02, blue: 0.05, alpha: 1)

        let cam = SCNCamera()
        cam.zNear = 0.05
        cam.zFar = 60
        cam.fieldOfView = 45
        cameraNode.camera = cam
        cameraNode.position = SCNVector3(3.6, 2.4, 3.8)
        cameraNode.look(at: SCNVector3(0.2, 0.3, -0.1))
        scene.rootNode.addChildNode(cameraNode)

        let ambient = SCNNode()
        ambient.light = SCNLight()
        ambient.light?.type = .ambient
        ambient.light?.intensity = 350
        scene.rootNode.addChildNode(ambient)

        let key = SCNNode()
        key.light = SCNLight()
        key.light?.type = .directional
        key.light?.intensity = 900
        key.eulerAngles = SCNVector3(-0.9, 0.6, 0)
        scene.rootNode.addChildNode(key)

        buildTurbine()
        buildFrontEngine()
        buildRearExhaust()
        buildDriveShaft()
        buildParticles()
    }

    private func buildTurbine() {
        scene.rootNode.addChildNode(diskSpin)

        let diskMaterial = metal(alpha: 0.6, white: 0.78)
        let solid = Self.diskGeometry(cfg, withHoles: false)
        let drilled = Self.diskGeometry(cfg, withHoles: true)
        solid.materials = [diskMaterial]
        drilled.materials = [diskMaterial]

        for i in 0..<cfg.diskCount {
            let node = SCNNode(geometry: i == 0 ? solid : drilled)
            node.eulerAngles.x = -.pi / 2
            node.position.y = sim.diskY(i)
            node.renderingOrder = 5
            diskSpin.addChildNode(node)
        }

        let shaftBottom: Float = -0.62
        let shaftTop = sim.diskY(cfg.diskCount - 1) + 0.05
        let shaft = SCNCylinder(radius: CGFloat(cfg.hubRadius), height: CGFloat(shaftTop - shaftBottom))
        shaft.firstMaterial = metal(alpha: 1, white: 0.35)
        let shaftNode = SCNNode(geometry: shaft)
        shaftNode.position.y = (shaftTop + shaftBottom) / 2
        diskSpin.addChildNode(shaftNode)

        let plateMat = SCNMaterial()
        plateMat.diffuse.contents = UIColor(red: 0.12, green: 0.14, blue: 0.2, alpha: 1)
        plateMat.transparency = 0.7

        let bottom = SCNCylinder(radius: CGFloat(cfg.diskRadius + 0.06), height: 0.015)
        bottom.firstMaterial = plateMat
        let bottomNode = SCNNode(geometry: bottom)
        bottomNode.position.y = sim.diskY(0) - 0.05
        scene.rootNode.addChildNode(bottomNode)

        let top = SCNTube(innerRadius: CGFloat(cfg.holeOuterRadius),
                          outerRadius: CGFloat(cfg.diskRadius + 0.06), height: 0.015)
        top.firstMaterial = plateMat
        let topNode = SCNNode(geometry: top)
        topNode.position.y = shaftTop - 0.0
        scene.rootNode.addChildNode(topNode)

        let stackH = CGFloat(Float(cfg.diskCount) * cfg.pitch)
        let shroud = SCNTube(innerRadius: CGFloat(cfg.diskRadius + 0.03),
                             outerRadius: CGFloat(cfg.diskRadius + 0.055), height: stackH)
        shroud.firstMaterial = metal(alpha: 0.12, white: 0.5)
        scene.rootNode.addChildNode(SCNNode(geometry: shroud))
    }

    private func buildFrontEngine() {
        let L = cfg.jetCellLength
        let J = Float(cfg.chainACells)
        func d(_ c: Float) -> Float { (J - c) * L }

        let engine = SCNNode()
        engine.position = SCNVector3(sim.jetX, 0, 0)
        scene.rootNode.addChildNode(engine)

        let casing = metal(alpha: 0.35)
        combustorMaterial.lightingModel = .blinn
        combustorMaterial.diffuse.contents = UIColor(red: 0.35, green: 0.2, blue: 0.15, alpha: 1)
        combustorMaterial.emission.contents = UIColor(red: 1.0, green: 0.4, blue: 0.08, alpha: 1)
        combustorMaterial.emission.intensity = 0
        combustorMaterial.isDoubleSided = true
        combustorMaterial.transparency = 0.35

        func zTube(radius: Float, from c0: Float, to c1: Float, material: SCNMaterial) {
            let cyl = SCNCylinder(radius: CGFloat(radius), height: CGFloat((c1 - c0) * L))
            cyl.firstMaterial = material
            let n = SCNNode(geometry: cyl)
            n.eulerAngles.x = .pi / 2
            n.position = SCNVector3(0, 0, (d(c0) + d(c1)) / 2)
            n.renderingOrder = 5
            engine.addChildNode(n)
        }

        zTube(radius: 0.19, from: 0, to: 2, material: casing)

        let comp = SCNCone(topRadius: 0.15, bottomRadius: 0.19, height: CGFloat(4 * L))
        comp.firstMaterial = casing
        let compNode = SCNNode(geometry: comp)
        compNode.eulerAngles.x = -.pi / 2
        compNode.position = SCNVector3(0, 0, (d(2) + d(6)) / 2)
        compNode.renderingOrder = 5
        engine.addChildNode(compNode)

        zTube(radius: 0.17, from: 6, to: 12, material: combustorMaterial)

        injectorMaterial.diffuse.contents = UIColor(white: 0.2, alpha: 1)
        injectorMaterial.emission.contents = UIColor(red: 1.0, green: 0.6, blue: 0.1, alpha: 1)
        injectorMaterial.emission.intensity = 0
        for c in [Float(7), 9, 11] {
            let inj = SCNCylinder(radius: 0.012, height: 0.14)
            inj.firstMaterial = injectorMaterial
            let n = SCNNode(geometry: inj)
            n.position = SCNVector3(0, 0.22, d(c))
            engine.addChildNode(n)
        }

        let len = CGFloat(4 * L)
        let slotPath = UIBezierPath()
        slotPath.move(to: CGPoint(x: 0, y: -CGFloat(sim.stackHalfSpan)))
        slotPath.addLine(to: CGPoint(x: 0, y: CGFloat(sim.stackHalfSpan)))
        slotPath.addLine(to: CGPoint(x: len, y: 0.155))
        slotPath.addLine(to: CGPoint(x: len, y: -0.155))
        slotPath.close()
        let slot = SCNShape(path: slotPath, extrusionDepth: 0.1)
        slot.firstMaterial = casing
        let slotNode = SCNNode(geometry: slot)
        slotNode.eulerAngles.y = -.pi / 2
        slotNode.renderingOrder = 5
        engine.addChildNode(slotNode)

        let nose = SCNCone(topRadius: 0, bottomRadius: 0.12, height: 0.16)
        nose.firstMaterial = metal(alpha: 1, white: 0.5)
        let noseNode = SCNNode(geometry: nose)
        noseNode.eulerAngles.x = .pi / 2
        noseNode.position = SCNVector3(0, 0, d(0) + 0.08)
        engine.addChildNode(noseNode)

        let fanHolder = SCNNode()
        fanHolder.eulerAngles.x = .pi / 2
        fanHolder.position = SCNVector3(0, 0, d(0) - 0.04)
        engine.addChildNode(fanHolder)
        fanHolder.addChildNode(fanSpin)
        for k in 0..<8 {
            let blade = SCNNode(geometry: SCNBox(width: 0.3, height: 0.004, length: 0.03, chamferRadius: 0))
            blade.geometry?.firstMaterial = metal(alpha: 1, white: 0.55)
            blade.eulerAngles.y = Float(k) * .pi / 8
            fanSpin.addChildNode(blade)
        }

        let rotorHolder = SCNNode()
        rotorHolder.eulerAngles.x = .pi / 2
        rotorHolder.position = SCNVector3(0, 0, (d(2) + d(6)) / 2)
        engine.addChildNode(rotorHolder)
        rotorHolder.addChildNode(compressorSpin)
        let hub = SCNCylinder(radius: 0.03, height: CGFloat(4 * L))
        hub.firstMaterial = metal(alpha: 1, white: 0.4)
        compressorSpin.addChildNode(SCNNode(geometry: hub))
        let stageY: [Float] = [-0.18, -0.06, 0.06, 0.18]
        let stageR: [Float] = [0.15, 0.14, 0.13, 0.12]
        for s in 0..<4 {
            for k in 0..<6 {
                let b = SCNBox(width: CGFloat(stageR[s] * 2), height: 0.008, length: 0.02, chamferRadius: 0)
                b.firstMaterial = metal(alpha: 1, white: 0.65)
                let n = SCNNode(geometry: b)
                n.position = SCNVector3(0, stageY[s], 0)
                n.eulerAngles.y = Float(k) * .pi / 6
                compressorSpin.addChildNode(n)
            }
        }
    }

    private func buildRearExhaust() {
        let mat = metal(alpha: 0.3)
        let topY = sim.diskY(cfg.diskCount - 1) + 0.05

        let funnel = SCNCone(topRadius: 0.24, bottomRadius: 0.58, height: CGFloat(0.80 - topY))
        funnel.firstMaterial = mat
        let funnelNode = SCNNode(geometry: funnel)
        funnelNode.position = SCNVector3(0, (0.80 + topY) / 2, 0)
        funnelNode.renderingOrder = 5
        scene.rootNode.addChildNode(funnelNode)

        let elbow = SCNSphere(radius: 0.25)
        elbow.firstMaterial = mat
        let elbowNode = SCNNode(geometry: elbow)
        elbowNode.position = SCNVector3(0, 0.80, 0)
        elbowNode.renderingOrder = 5
        scene.rootNode.addChildNode(elbowNode)

        let nozzle = SCNCone(topRadius: 0.105, bottomRadius: 0.225, height: 1.6)
        nozzle.firstMaterial = mat
        let nozzleNode = SCNNode(geometry: nozzle)
        nozzleNode.eulerAngles.x = -.pi / 2
        nozzleNode.position = SCNVector3(0, 0.80, -1.15)
        nozzleNode.renderingOrder = 5
        scene.rootNode.addChildNode(nozzleNode)

        let lip = SCNTube(innerRadius: 0.09, outerRadius: 0.125, height: 0.03)
        lip.firstMaterial = metal(alpha: 1, white: 0.5)
        let lipNode = SCNNode(geometry: lip)
        lipNode.eulerAngles.x = .pi / 2
        lipNode.position = SCNVector3(0, 0.80, -1.95)
        scene.rootNode.addChildNode(lipNode)
    }

    private func buildDriveShaft() {
        let mat = metal(alpha: 1, white: 0.4)
        let y: Float = -0.62
        let x = sim.jetX
        let L = cfg.jetCellLength
        let zc = (Float(cfg.chainACells - 2) + Float(cfg.chainACells - 6)) * L / 2

        func rod(_ node: SCNNode, radius: CGFloat, height: CGFloat) -> SCNNode {
            let cyl = SCNCylinder(radius: radius, height: height)
            cyl.firstMaterial = mat
            node.geometry = cyl
            return node
        }
        let a = rod(SCNNode(), radius: 0.03, height: CGFloat(x))
        a.eulerAngles.z = .pi / 2
        a.position = SCNVector3(x / 2, y, 0)
        scene.rootNode.addChildNode(a)
        let b = rod(SCNNode(), radius: 0.03, height: CGFloat(zc))
        b.eulerAngles.x = .pi / 2
        b.position = SCNVector3(x, y, zc / 2)
        scene.rootNode.addChildNode(b)
        let c = rod(SCNNode(), radius: 0.03, height: CGFloat(-y))
        c.position = SCNVector3(x, y / 2, zc)
        scene.rootNode.addChildNode(c)

        for p in [SCNVector3(0, y, 0), SCNVector3(x, y, 0), SCNVector3(x, y, zc), SCNVector3(x, 0, zc)] {
            let box = SCNBox(width: 0.1, height: 0.1, length: 0.1, chamferRadius: 0.02)
            box.firstMaterial = metal(alpha: 1, white: 0.3)
            let n = SCNNode(geometry: box)
            n.position = p
            scene.rootNode.addChildNode(n)
        }
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
        scene.rootNode.addChildNode(holder)
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

    private func band(for temp: Float) -> Int8 {
        temp > 1200 ? 0 : temp > 900 ? 1 : temp > 600 ? 2 : 3
    }

    func renderer(_ renderer: SCNSceneRenderer, updateAtTime time: TimeInterval) {
        if lastTime == 0 { lastTime = time }
        let frame = min(Float(time - lastTime), 0.1)
        lastTime = time

        accumulator += frame
        let tickDt = 1 / cfg.tickRate
        while accumulator >= tickDt {
            sim.tick()
            accumulator -= tickDt
        }
        let alpha = accumulator / tickDt

        let angle = sim.diskAngle + sim.omega * accumulator
        diskSpin.eulerAngles.y = angle
        compressorSpin.eulerAngles.y = angle
        fanSpin.eulerAngles.y = angle

        combustorMaterial.emission.intensity = CGFloat(max(0, min(1, (sim.flameTemp - 300) / 1500)))
        injectorMaterial.emission.intensity = CGFloat(sim.fuel)

        let particles = sim.particles
        for i in 0..<particles.count {
            let p = particles[i]
            let node = particleNodes[i]
            if p.stage == .free {
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
            }
        }
    }
}

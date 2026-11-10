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

    private let engineRoot = SCNNode()
    private let turbineSpinNode = SCNNode()
    private let compressorSpinNode = SCNNode()
    private let fanSpinNode = SCNNode()
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
    func setPressureRatio(_ v: Float) { sim.hpcPressureRatio = v }
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
        cameraNode.position = SCNVector3(0, 1.8, 4.5)
        cameraNode.look(at: SCNVector3(0, 0, -0.2))
        scene.rootNode.addChildNode(cameraNode)

        let ambient = SCNNode()
        ambient.light = SCNLight()
        ambient.light?.type = .ambient
        ambient.light?.intensity = 400
        scene.rootNode.addChildNode(ambient)

        let key = SCNNode()
        key.light = SCNLight()
        key.light?.type = .directional
        key.light?.intensity = 1000
        key.eulerAngles = SCNVector3(-0.8, 0.5, 0)
        scene.rootNode.addChildNode(key)

        scene.rootNode.addChildNode(engineRoot)

        buildJetEngineBody()
        buildParticles()
    }

    private func buildJetEngineBody() {
        let casingMat = metal(alpha: 0.35, white: 0.7)
        combustorMaterial.lightingModel = .blinn
        combustorMaterial.diffuse.contents = UIColor(red: 0.35, green: 0.2, blue: 0.15, alpha: 1)
        combustorMaterial.emission.contents = UIColor(red: 1.0, green: 0.4, blue: 0.08, alpha: 1)
        combustorMaterial.emission.intensity = 0
        combustorMaterial.isDoubleSided = true
        combustorMaterial.transparency = 0.35

        // Fan Cowling
        let intakeCyl = SCNCylinder(radius: 0.38, height: 0.4)
        intakeCyl.firstMaterial = casingMat
        let intakeNode = SCNNode(geometry: intakeCyl)
        intakeNode.eulerAngles.x = .pi / 2
        intakeNode.position = SCNVector3(0, 0, 1.4)
        engineRoot.addChildNode(intakeNode)

        // Nose Cone
        let noseCone = SCNCone(topRadius: 0, bottomRadius: 0.14, height: 0.3)
        noseCone.firstMaterial = metal(alpha: 1, white: 0.8)
        let noseNode = SCNNode(geometry: noseCone)
        noseNode.eulerAngles.x = .pi / 2
        noseNode.position = SCNVector3(0, 0, 1.5)
        engineRoot.addChildNode(noseNode)

        // Fan Blades
        let fanHolder = SCNNode()
        fanHolder.position = SCNVector3(0, 0, 1.25)
        engineRoot.addChildNode(fanHolder)
        fanHolder.addChildNode(fanSpinNode)
        for k in 0..<16 {
            let blade = SCNNode(geometry: SCNBox(width: 0.34, height: 0.006, length: 0.03, chamferRadius: 0))
            blade.geometry?.firstMaterial = metal(alpha: 1, white: 0.6)
            blade.eulerAngles.z = Float(k) * 2 * .pi / 16
            fanSpinNode.addChildNode(blade)
        }

        // Low-Pressure & High-Pressure Compressor Stages
        let compCyl = SCNCylinder(radius: 0.28, height: 1.0)
        compCyl.firstMaterial = casingMat
        let compNode = SCNNode(geometry: compCyl)
        compNode.eulerAngles.x = .pi / 2
        compNode.position = SCNVector3(0, 0, 0.55)
        engineRoot.addChildNode(compNode)

        let compRotorHolder = SCNNode()
        compRotorHolder.position = SCNVector3(0, 0, 0.55)
        engineRoot.addChildNode(compRotorHolder)
        compRotorHolder.addChildNode(compressorSpinNode)

        // LPC blades (front) + HPC blades (rear)
        for s in 0..<7 {
            let zOffset = Float(s) * 0.14 - 0.42
            let bladeCount = s < 3 ? 8 : 10
            for k in 0..<bladeCount {
                let blade = SCNNode(geometry: SCNBox(width: s < 3 ? 0.24 : 0.18, height: 0.005, length: 0.015, chamferRadius: 0))
                blade.geometry?.firstMaterial = metal(alpha: 1, white: 0.75)
                blade.position = SCNVector3(0, 0, zOffset)
                blade.eulerAngles.z = Float(k) * 2 * .pi / Float(bladeCount) + Float(s) * 0.2
                compressorSpinNode.addChildNode(blade)
            }
        }

        // Combustion Chamber
        let combustorCyl = SCNCylinder(radius: 0.32, height: 0.45)
        combustorCyl.firstMaterial = combustorMaterial
        let combNode = SCNNode(geometry: combustorCyl)
        combNode.eulerAngles.x = .pi / 2
        combNode.position = SCNVector3(0, 0, -0.05)
        engineRoot.addChildNode(combNode)

        // High-Pressure & Low-Pressure Turbines
        engineRoot.addChildNode(turbineSpinNode)
        turbineSpinNode.position = SCNVector3(0, 0, -0.45)

        let diskMat = metal(alpha: 0.85, white: 0.9)
        let solidDisk = Self.diskGeometry(cfg, withHoles: false)
        let drilledDisk = Self.diskGeometry(cfg, withHoles: true)
        solidDisk.materials = [diskMat]
        drilledDisk.materials = [diskMat]

        for i in 0..<cfg.diskCount {
            let node = SCNNode(geometry: i < 3 ? solidDisk : drilledDisk)
            node.position.z = sim.diskZ(i)
            node.renderingOrder = 5
            turbineSpinNode.addChildNode(node)
        }

        // Coaxial Shafts (Low-Pressure & High-Pressure Shafts)
        let outerShaftCyl = SCNCylinder(radius: CGFloat(cfg.hubRadius * 1.3), height: 2.2)
        outerShaftCyl.firstMaterial = metal(alpha: 1, white: 0.35)
        let outerShaftNode = SCNNode(geometry: outerShaftCyl)
        outerShaftNode.eulerAngles.x = .pi / 2
        outerShaftNode.position = SCNVector3(0, 0, 0.2)
        engineRoot.addChildNode(outerShaftNode)

        let innerShaftCyl = SCNCylinder(radius: CGFloat(cfg.hubRadius * 0.6), height: 2.6)
        innerShaftCyl.firstMaterial = metal(alpha: 1, white: 0.5)
        let innerShaftNode = SCNNode(geometry: innerShaftCyl)
        innerShaftNode.eulerAngles.x = .pi / 2
        innerShaftNode.position = SCNVector3(0, 0, 0.2)
        engineRoot.addChildNode(innerShaftNode)

        // Exhaust Shroud & Nozzle
        let exhaustShroud = SCNCone(topRadius: 0.32, bottomRadius: 0.18, height: 0.9)
        exhaustShroud.firstMaterial = metal(alpha: 0.4, white: 0.6)
        let exhaustNode = SCNNode(geometry: exhaustShroud)
        exhaustNode.eulerAngles.x = -.pi / 2
        exhaustNode.position = SCNVector3(0, 0, -1.35)
        engineRoot.addChildNode(exhaustNode)

        let exhaustCone = SCNCone(topRadius: 0.16, bottomRadius: 0.0, height: 0.7)
        exhaustCone.firstMaterial = metal(alpha: 1, white: 0.5)
        let exhaustConeNode = SCNNode(geometry: exhaustCone)
        exhaustConeNode.eulerAngles.x = .pi / 2
        exhaustConeNode.position = SCNVector3(0, 0, -1.3)
        engineRoot.addChildNode(exhaustConeNode)
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
        turbineSpinNode.eulerAngles.z = angle
        compressorSpinNode.eulerAngles.z = angle * 1.15 // High-spool differential RPM scaling
        fanSpinNode.eulerAngles.z = angle * 0.85

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

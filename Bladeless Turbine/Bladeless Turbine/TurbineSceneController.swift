//
//  TurbineSceneController.swift
//  Bladeless Turbine
//
//  Builds the SceneKit scene (disk stack with three openings per disk, shaft,
//  end plates, nozzle, glowing gas particles) and drives the CA each frame.
//

import SwiftUI
import SceneKit
import UIKit
import Combine

final class TurbineSceneController: NSObject, SCNSceneRendererDelegate, ObservableObject {

    // HUD state (published at ~8 Hz)
    @Published var rpm: Float = 0
    @Published var activeCount = 0
    @Published var exhaustedTotal = 0

    let cfg: TurbineConfig
    let sim: TurbineCA
    let scene = SCNScene()
    let cameraNode = SCNNode()

    private let diskSpin = SCNNode()
    private var particleNodes: [SCNNode] = []
    private var lastBand: [Int8]
    private var bandGeometries: [SCNGeometry] = []

    private var lastTime: TimeInterval = 0
    private var accumulator: Float = 0
    private var hudTimer: Float = 0

    // MARK: Init

    init(config: TurbineConfig = TurbineConfig()) {
        cfg = config
        sim = TurbineCA(config: config)
        lastBand = Array(repeating: -1, count: config.maxParticles)
        super.init()
        buildScene()
    }

    func setJetIntensity(_ v: Float) { sim.jetIntensity = v }

    func reset() {
        sim.reset()
        accumulator = 0
        for n in particleNodes { n.isHidden = true }
    }

    // MARK: Scene construction

    private func buildScene() {
        scene.background.contents = UIColor(red: 0.02, green: 0.02, blue: 0.05, alpha: 1)

        // Camera
        let cam = SCNCamera()
        cam.zNear = 0.05
        cam.zFar = 50
        cam.fieldOfView = 45
        cameraNode.camera = cam
        cameraNode.position = SCNVector3(2.3, 1.5, 2.6)
        cameraNode.look(at: SCNVector3Zero)
        scene.rootNode.addChildNode(cameraNode)

        // Lights
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

        // Disks (spinning group). Disk 0 is a solid end cap, the rest have openings.
        scene.rootNode.addChildNode(diskSpin)
        let diskMaterial = SCNMaterial()
        diskMaterial.lightingModel = .physicallyBased
        diskMaterial.diffuse.contents = UIColor(white: 0.78, alpha: 1)
        diskMaterial.metalness.contents = 0.9
        diskMaterial.roughness.contents = 0.35
        diskMaterial.isDoubleSided = true
        diskMaterial.transparency = 0.6

        let solid = Self.diskGeometry(cfg, withHoles: false)
        let drilled = Self.diskGeometry(cfg, withHoles: true)
        solid.materials = [diskMaterial]
        drilled.materials = [diskMaterial]

        for i in 0..<cfg.diskCount {
            let node = SCNNode(geometry: i == 0 ? solid : drilled)
            node.eulerAngles.x = -.pi / 2          // shape XY plane → world XZ, extrude along +Y
            node.position.y = sim.diskY(i)
            node.renderingOrder = 5
            diskSpin.addChildNode(node)
        }

        // Shaft
        let stackH = Float(cfg.diskCount) * cfg.pitch + 0.3
        let shaft = SCNCylinder(radius: CGFloat(cfg.hubRadius), height: CGFloat(stackH))
        shaft.firstMaterial?.diffuse.contents = UIColor(white: 0.35, alpha: 1)
        shaft.firstMaterial?.metalness.contents = 1.0
        shaft.firstMaterial?.lightingModel = .physicallyBased
        diskSpin.addChildNode(SCNNode(geometry: shaft))

        // Stationary end plates (top plate is a ring: its open centre is the exhaust port)
        let plateMat = SCNMaterial()
        plateMat.diffuse.contents = UIColor(red: 0.12, green: 0.14, blue: 0.2, alpha: 1)
        plateMat.transparency = 0.7

        let bottom = SCNCylinder(radius: CGFloat(cfg.diskRadius + 0.06), height: 0.015)
        bottom.firstMaterial = plateMat
        let bottomNode = SCNNode(geometry: bottom)
        bottomNode.position.y = sim.diskY(0) - 0.05
        scene.rootNode.addChildNode(bottomNode)

        let top = SCNTube(innerRadius: CGFloat(cfg.holeOuterRadius),
                          outerRadius: CGFloat(cfg.diskRadius + 0.06),
                          height: 0.015)
        top.firstMaterial = plateMat
        let topNode = SCNNode(geometry: top)
        topNode.position.y = sim.diskY(cfg.diskCount - 1) + 0.05
        scene.rootNode.addChildNode(topNode)

        // Nozzle: a slot along the rim aimed tangentially (+θ at θ = 0 is world −Z).
        let stackSpan = CGFloat(Float(cfg.gapCount) * cfg.pitch)
        let nozzle = SCNBox(width: 0.05, height: stackSpan, length: 0.22, chamferRadius: 0.01)
        let nozzleMat = SCNMaterial()
        nozzleMat.diffuse.contents = UIColor(red: 0.25, green: 0.1, blue: 0.05, alpha: 1)
        nozzleMat.emission.contents = UIColor(red: 1.0, green: 0.45, blue: 0.1, alpha: 1)
        nozzleMat.emission.intensity = 0.6
        nozzle.firstMaterial = nozzleMat
        let nozzleNode = SCNNode(geometry: nozzle)
        nozzleNode.position = SCNVector3(cfg.diskRadius + 0.08, 0, 0.12)
        scene.rootNode.addChildNode(nozzleNode)

        buildParticles()
    }

    private func buildParticles() {
        let colors: [UIColor] = [
            UIColor(red: 1.00, green: 0.95, blue: 0.70, alpha: 1),   // hottest
            UIColor(red: 1.00, green: 0.60, blue: 0.15, alpha: 1),
            UIColor(red: 0.90, green: 0.20, blue: 0.08, alpha: 1),
            UIColor(red: 0.35, green: 0.45, blue: 0.70, alpha: 1)    // spent
        ]
        for c in colors {
            let s = SCNSphere(radius: 0.013)
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
            let n = SCNNode(geometry: bandGeometries[0])
            n.isHidden = true
            n.renderingOrder = 20
            holder.addChildNode(n)
            particleNodes.append(n)
        }
        scene.rootNode.addChildNode(holder)
    }

    // MARK: Disk geometry – annulus with a shaft bore and three sector openings

    static func diskGeometry(_ cfg: TurbineConfig, withHoles: Bool) -> SCNShape {
        let path = UIBezierPath()
        let outer = CGFloat(cfg.diskRadius)
        let bore = CGFloat(cfg.hubRadius)

        // Outer rim: increasing angle. Bore and openings wind the other way so
        // SceneKit treats them as holes.
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

    // MARK: Render loop

    private func band(for temp: Float) -> Int8 {
        temp > 0.75 ? 0 : temp > 0.5 ? 1 : temp > 0.3 ? 2 : 3
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

        // Disks rotate with the CA's disk angle so the drawn openings match the rule set.
        diskSpin.eulerAngles.y = sim.diskAngle + sim.diskOmega * accumulator

        let particles = sim.particles
        for i in 0..<particles.count {
            let p = particles[i]
            let node = particleNodes[i]
            if !p.active {
                if !node.isHidden { node.isHidden = true }
                continue
            }
            node.isHidden = false
            node.position = sim.position(of: p, alpha: alpha)
            let b = band(for: p.temp)
            if b != lastBand[i] {
                node.geometry = bandGeometries[Int(b)]
                lastBand[i] = b
            }
        }

        hudTimer += frame
        if hudTimer > 0.12 {
            hudTimer = 0
            let rpm = sim.rpm, active = sim.activeCount, out = sim.exhaustedTotal
            DispatchQueue.main.async {
                self.rpm = rpm
                self.activeCount = active
                self.exhaustedTotal = out
            }
        }
    }
}

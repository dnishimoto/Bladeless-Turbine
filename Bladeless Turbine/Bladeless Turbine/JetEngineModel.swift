//
//  File.swift
//  Bladeless Turbine
//
//  Created by David Nishimoto on 11/9/26.
//

import Foundation
import SwiftUI
import SceneKit
import Combine




// MARK: - Engine Simulation Engine

final class JetEngineModel: NSObject, SCNSceneRendererDelegate, ObservableObject {
    @Published var rpm: Double = 0
    @Published var thrust: Double = 0
    @Published var exhaustVelocity: Double = 0
    @Published var pressureRatio: Double = 1
    @Published var coreTemperature: Double = 288.15
    @Published var activeParticles: Int = 0
    @Published var totalExhausted: Int = 0
    @Published var currentStageText: String = "AIR INTAKE"

    @Published var throttle: Double = 0.8 {
        didSet { config.fuelFlowRate = Float(throttle) * 0.08 }
    }
    @Published var compressorRatioSetting: Double = 4.0 {
        didSet { config.compressorRatio = Float(compressorRatioSetting) }
    }

    var config = EngineConfiguration()
    private var particles: [JetGasParticle] = []
    private var freeIDs: [Int] = []

    let scene = SCNScene()
    let cameraNode = SCNNode()
    private let diskAssembly = SCNNode()
    private let particleContainer = SCNNode()

    private var diskAngle: Float = 0
    private var diskOmega: Float = 0
    private var totalTorque: Float = 0
    private var exhaustedCount = 0

    private var lastTime: TimeInterval = 0
    private var accumulator: Float = 0
    private var injectionTimer: Float = 0

    override init() {
        super.init()
        particles = Array(repeating: JetGasParticle(), count: config.maxParticles)
        freeIDs = Array((0..<config.maxParticles).reversed())
        setupScene()
    }

    func reset() {
        particles = Array(repeating: JetGasParticle(), count: config.maxParticles)
        freeIDs = Array((0..<config.maxParticles).reversed())
        diskAngle = 0
        diskOmega = 0
        totalTorque = 0
        exhaustedCount = 0
        for node in particleContainer.childNodes {
            node.isHidden = true
        }
    }

    private func setupScene() {
        scene.background.contents = UIColor(red: 0.01, green: 0.01, blue: 0.03, alpha: 1)

        let cam = SCNCamera()
        cam.zNear = 0.05
        cam.zFar = 60
        cam.fieldOfView = 48
        cameraNode.camera = cam
        cameraNode.position = SCNVector3(3.5, 2.2, 4.2)
        cameraNode.look(at: SCNVector3(0, 0.5, 0))
        scene.rootNode.addChildNode(cameraNode)

        let ambient = SCNNode()
        ambient.light = SCNLight()
        ambient.light?.type = .ambient
        ambient.light?.intensity = 400
        scene.rootNode.addChildNode(ambient)

        let directional = SCNNode()
        directional.light = SCNLight()
        directional.light?.type = .directional
        directional.light?.intensity = 1000
        directional.eulerAngles = SCNVector3(-0.8, 0.5, 0)
        scene.rootNode.addChildNode(directional)

        let root = SCNNode()
        scene.rootNode.addChildNode(root)

        buildIntake(root)
        buildCompressor(root)
        buildCombustionChamber(root)
        buildTangentialNozzle(root)
        buildTeslaTurbineStack(root)
        buildExhaustCollector(root)

        particleContainer.name = "GasParticles"
        root.addChildNode(particleContainer)
    }

    private func pbrMaterial(_ color: UIColor, metallic: CGFloat = 0.2, roughness: CGFloat = 0.3, transparency: CGFloat = 1.0) -> SCNMaterial {
        let m = SCNMaterial()
        m.lightingModel = .physicallyBased
        m.diffuse.contents = color
        m.metalness.contents = metallic
        m.roughness.contents = roughness
        m.transparency = transparency
        m.isDoubleSided = true
        return m
    }

    private func buildIntake(_ root: SCNNode) {
        let tube = SCNTube(innerRadius: 0.35, outerRadius: 0.55, height: 0.6)
        tube.firstMaterial = pbrMaterial(.gray, metallic: 0.8, roughness: 0.2)
        let node = SCNNode(geometry: tube)
        node.position = SCNVector3(0, 2.8, 0)
        root.addChildNode(node)
    }

    private func buildCompressor(_ root: SCNNode) {
        let tube = SCNTube(innerRadius: 0.30, outerRadius: 0.62, height: 0.5)
        tube.firstMaterial = pbrMaterial(.systemBlue, metallic: 0.7, roughness: 0.3)
        let node = SCNNode(geometry: tube)
        node.position = SCNVector3(0, 2.15, 0)
        root.addChildNode(node)

        let rotor = SCNCylinder(radius: 0.28, height: 0.15)
        rotor.firstMaterial = pbrMaterial(.cyan, metallic: 0.9)
        let rotorNode = SCNNode(geometry: rotor)
        rotorNode.position = SCNVector3(0, 2.15, 0)
        root.addChildNode(rotorNode)
    }

    private func buildCombustionChamber(_ root: SCNNode) {
        let chamber = SCNTube(innerRadius: 0.28, outerRadius: 0.60, height: 0.5)
        chamber.firstMaterial = pbrMaterial(.systemOrange, metallic: 0.4, roughness: 0.4)
        let node = SCNNode(geometry: chamber)
        node.position = SCNVector3(0, 1.5, 0)
        root.addChildNode(node)

        let core = SCNSphere(radius: 0.22)
        core.firstMaterial = pbrMaterial(.orange, metallic: 0.1, roughness: 0.1)
        core.firstMaterial?.emission.contents = UIColor.orange
        let coreNode = SCNNode(geometry: core)
        coreNode.position = SCNVector3(0, 1.5, 0)
        root.addChildNode(coreNode)
    }

    private func buildTangentialNozzle(_ root: SCNNode) {
        let nozzle = SCNCylinder(radius: 0.08, height: 0.9)
        nozzle.firstMaterial = pbrMaterial(.systemRed, metallic: 0.8)
        let node = SCNNode(geometry: nozzle)
        node.eulerAngles.z = .pi / 2
        node.position = SCNVector3(0.85, 0.75, 0)
        root.addChildNode(node)
    }

    private func buildTeslaTurbineStack(_ root: SCNNode) {
        diskAssembly.name = "TeslaDiscs"
        root.addChildNode(diskAssembly)

        let solidDisk = createDiskShape(withHoles: false)
        let drilledDisk = createDiskShape(withHoles: true)
        let mat = pbrMaterial(UIColor(white: 0.75, alpha: 1), metallic: 0.95, roughness: 0.25, transparency: 0.7)
        solidDisk.materials = [mat]
        drilledDisk.materials = [mat]

        let startY = -Float(config.diskCount - 1) * config.diskSpacing / 2
        for i in 0..<config.diskCount {
            let node = SCNNode(geometry: i == 0 ? solidDisk : drilledDisk)
            node.eulerAngles.x = -.pi / 2
            node.position.y = startY + Float(i) * config.diskSpacing
            node.renderingOrder = 5
            diskAssembly.addChildNode(node)
        }

        let stackH = CGFloat(config.diskCount) * CGFloat(config.diskSpacing) + 0.2
        let shaft = SCNCylinder(radius: CGFloat(config.hubRadius), height: stackH)
        shaft.firstMaterial = pbrMaterial(.darkGray, metallic: 1.0)
        diskAssembly.addChildNode(SCNNode(geometry: shaft))
    }

    private func createDiskShape(withHoles: Bool) -> SCNShape {
        let path = UIBezierPath()
        let outer = CGFloat(config.diskRadius)
        let bore = CGFloat(config.hubRadius)

        path.append(UIBezierPath(arcCenter: .zero, radius: outer, startAngle: 0, endAngle: 2 * .pi, clockwise: true))
        path.append(UIBezierPath(arcCenter: .zero, radius: bore, startAngle: 2 * .pi, endAngle: 0, clockwise: false))

        if withHoles {
            let rIn = CGFloat(config.holeInnerRadius)
            let rOut = CGFloat(config.holeOuterRadius)
            let half = CGFloat(config.holeHalfAngle)
            for k in 0..<config.holeCount {
                let c = CGFloat(k) * 2 * .pi / CGFloat(config.holeCount)
                let hole = UIBezierPath()
                hole.addArc(withCenter: .zero, radius: rOut, startAngle: c + half, endAngle: c - half, clockwise: false)
                hole.addArc(withCenter: .zero, radius: rIn, startAngle: c - half, endAngle: c + half, clockwise: true)
                hole.close()
                path.append(hole)
            }
        }
        path.usesEvenOddFillRule = true
        let shape = SCNShape(path: path, extrusionDepth: CGFloat(config.diskThickness))
        return shape
    }

    private func buildExhaustCollector(_ root: SCNNode) {
        let exhaustTube = SCNTube(innerRadius: 0.12, outerRadius: 0.2, height: 1.6)
        exhaustTube.firstMaterial = pbrMaterial(.darkGray, metallic: 0.8)
        let node = SCNNode(geometry: exhaustTube)
        node.position = SCNVector3(0, -1.2, 0)
        root.addChildNode(node)
    }

    func renderer(_ renderer: SCNSceneRenderer, updateAtTime time: TimeInterval) {
        if lastTime == 0 { lastTime = time }
        let dt = min(Float(time - lastTime), 0.05)
        lastTime = time

        accumulator += dt
        let tickDt = 1.0 / config.tickRate
        while accumulator >= tickDt {
            tickSimulation(dt: tickDt)
            accumulator -= tickDt
        }

        diskAngle = (diskAngle + diskOmega * dt).truncatingRemainder(dividingBy: 2 * .pi)
        diskAssembly.eulerAngles.y = diskAngle

        updateParticleNodes()

        if time - Double(Int(time)) < 0.05 {
            DispatchQueue.main.async {
                self.updateTelemetry()
            }
        }
    }

    private func tickSimulation(dt: Float) {
        injectionTimer += dt
        if injectionTimer >= 0.04 {
            injectionTimer = 0
            injectParticles()
        }

        var accumulatedTorque: Float = 0
        let activeIndices = particles.indices.filter { particles[$0].active }

        for i in activeIndices {
            accumulatedTorque += updateParticle(&particles[i], dt: dt)
        }

        let alpha = accumulatedTorque * config.torqueGain - config.diskFriction * diskOmega
        diskOmega = max(0, diskOmega + alpha * dt)
        totalTorque = accumulatedTorque
    }

    private func injectParticles() {
        guard let id = freeIDs.last else { return }
        freeIDs.removeLast()

        var p = JetGasParticle()
        p.active = true
        p.stage = .intake
        p.axialPosition = 3.2
        p.radius = Float.random(in: 0.35...0.5)
        p.angle = Float.random(in: 0...(2 * .pi))
        p.temperature = config.ambientTemp
        p.pressure = config.ambientPressure
        p.velocity = 40.0

        let sphere = SCNSphere(radius: 0.02)
        sphere.segmentCount = 6
        let mat = SCNMaterial()
        mat.lightingModel = .constant
        mat.diffuse.contents = UIColor.cyan
        mat.blendMode = .add
        mat.writesToDepthBuffer = false
        sphere.firstMaterial = mat

        let node = SCNNode(geometry: sphere)
        particleContainer.addChildNode(node)
        p.node = node

        particles[id] = p
    }

    private func updateParticle(_ p: inout JetGasParticle, dt: Float) -> Float {
        p.age += dt
        var torqueContribution: Float = 0

        switch p.stage {
        case .intake:
            p.axialPosition -= p.velocity * dt * 0.4
            p.temperature = config.ambientTemp
            p.pressure = config.ambientPressure
            if p.axialPosition < 2.4 { p.stage = .compression }

        case .compression:
            p.axialPosition -= p.velocity * dt * 0.35
            let exponent = (config.gamma - 1.0) / config.gamma
            p.pressure = config.ambientPressure * config.compressorRatio
            p.temperature = config.ambientTemp * pow(config.compressorRatio, exponent)
            if p.axialPosition < 1.75 { p.stage = .combustion }

        case .combustion:
            p.axialPosition -= p.velocity * dt * 0.3
            let heatAddition = config.fuelFlowRate * 12000.0 * config.combustionEfficiency
            p.temperature = min(2200.0, p.temperature + heatAddition * dt)
            p.pressure *= 1.02
            if p.axialPosition < 1.0 {
                p.stage = .turbine
                p.radius = config.diskRadius * 0.95
                p.angularVelocity = 8.0
            }

        case .turbine:
            let slip = p.angularVelocity - diskOmega
            let shearForce = config.viscousCoupling * slip
            torqueContribution = abs(shearForce) * p.radius * (p.temperature / config.ambientTemp)

            p.angularVelocity -= shearForce * dt
            p.radius = max(config.hubRadius + 0.02, p.radius - config.radialFlowRate * dt)
            p.angle += p.angularVelocity * dt

            p.temperature = max(config.ambientTemp, p.temperature - 40.0 * dt)
            p.pressure = max(config.ambientPressure, p.pressure - 15.0 * dt)

            if p.radius <= config.hubRadius + 0.03 {
                p.stage = .exhaust
                p.axialPosition = -0.5
            }

        case .exhaust:
            p.axialPosition -= 2.5 * dt
            if p.axialPosition < -2.2 {
                p.active = false
                p.node?.removeFromParentNode()
                p.node = nil
                if let idx = particles.firstIndex(where: { $0.age == p.age }) {
                    freeIDs.append(idx)
                }
                exhaustedCount += 1
            }
        }

        return torqueContribution
    }

    private func updateParticleNodes() {
        for p in particles where p.active {
            guard let node = p.node else { continue }
            let x: Float, y: Float, z: Float

            if p.stage == .turbine {
                x = p.radius * cos(p.angle)
                z = p.radius * sin(p.angle)
                y = 0.2 * sin(p.angle * 2)
            } else {
                x = p.radius * cos(p.angle)
                z = p.radius * sin(p.angle)
                y = p.axialPosition
            }

            node.position = SCNVector3(x, y, z)

            let heatFactor = min(1.0, max(0.0, (p.temperature - 300.0) / 1500.0))
            let color = UIColor(red: 1.0, green: CGFloat(1.0 - heatFactor), blue: CGFloat(1.0 - heatFactor * 0.8), alpha: 1.0)
            node.geometry?.firstMaterial?.diffuse.contents = color
        }
    }

    private func updateTelemetry() {
        rpm = Double(diskOmega * 60.0 / (2.0 * .pi))
        pressureRatio = Double(config.compressorRatio)

        let active = particles.filter { $0.active }
        activeParticles = active.count
        totalExhausted = exhaustedCount

        coreTemperature = active.isEmpty ? Double(config.ambientTemp) : Double(active.map { $0.temperature }.reduce(0, +) / Float(active.count))

        let exhaustGasTemp = coreTemperature
        let exitVelocity = sqrt(max(0.0, 2.0 * 287.05 * exhaustGasTemp * (1.0 - pow(1.0 / max(1.1, pressureRatio), 0.286))))
        exhaustVelocity = exitVelocity
        thrust = exitVelocity * Double(config.fuelFlowRate + 0.15) * 15.0

        currentStageText = active.last?.stage.title ?? "IDLE"
    }
}

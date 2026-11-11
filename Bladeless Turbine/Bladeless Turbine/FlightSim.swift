//
//  FlightSim.swift
//  Bladeless Turbine
//
//  Point-mass flight model for the jet plus the procedural terrain it flies over.
//
//  Engine axis = +X (forward), +Y up, +Z right.  World units are metres.
//
//  Longitudinal:  m·dV/dt = F_net(engine cycle) − D − m·g·sin γ
//                 D = q·S·C_D0 + k·W²/(q·S)       (level-flight lift L = W)
//  Turning:       ψ̇ = g·tan φ / V                (coordinated turn)
//  Atmosphere:    ISA, so altitude and speed feed back into the engine cycle.
//
//  Terrain safety: (1) ground-collision avoidance auto-pulls up when the projected
//  path (1, 2, 3 s ahead) comes within `safeClearance` of the ground, and
//  (2) a hard floor keeps the jet above the terrain surface at all times.
//

import Foundation
import simd
import SceneKit
import UIKit


final class FlightSim {
    let cfg: TurbineConfig

    var position = SIMD3<Float>(0, 900, 0)
    var orientation = simd_quatf(angle: 0, axis: SIMD3<Float>(0, 1, 0))   // nose along +X
    var speed: Float = 170

    /// Pilot inputs, −1…1:  roll right +, pitch up +, rudder right +.
    var stickRoll: Float = 0
    var stickPitch: Float = 0
    var rudder: Float = 0

    private(set) var groundAvoid = false
    private(set) var clearance: Float = 900
    private(set) var drag: Float = 0

    static let safeClearance: Float = 140
    static let floorClearance: Float = 7
    private let gravity: Float = 9.80665

    init(config: TurbineConfig) { cfg = config }

    var forward: SIMD3<Float> { orientation.act(SIMD3(1, 0, 0)) }
    var up: SIMD3<Float> { orientation.act(SIMD3(0, 1, 0)) }
    var right: SIMD3<Float> { orientation.act(SIMD3(0, 0, 1)) }
    var altitude: Float { position.y }
    var bank: Float { asin(max(-1, min(1, -right.y))) }
    var pitchAngle: Float { asin(max(-1, min(1, forward.y))) }
    var heading: Float {
        var h = atan2(-forward.z, forward.x) * 180 / .pi
        if h < 0 { h += 360 }
        return h
    }
    var mach: Float {
        let t = Float(EngineEquations.atmosphere(altitude: Double(position.y)).t)
        return speed / (cfg.gamma * cfg.gasConstant * t).squareRoot()
    }

    func reset() {
        position = SIMD3(0, Terrain.height(0, 0) + 700, 0)
        orientation = simd_quatf(angle: 0, axis: SIMD3(0, 1, 0))
        speed = 170
        stickRoll = 0; stickPitch = 0; rudder = 0
        groundAvoid = false
    }

    private func projectedClearance() -> Float {
        var lowest = Float.greatestFiniteMagnitude
        let f = forward
        for t in [0.5, 1.0, 2.0, 3.0] as [Float] {
            let p = position + f * (max(60, speed) * t)
            lowest = min(lowest, p.y - Terrain.height(p.x, p.z))
        }
        return lowest
    }

    /// Advance by `dt`; `thrust` is the engine's net force (N) from the cycle.
    func step(dt: Float, thrust: Float) {
        let atm = EngineEquations.atmosphere(altitude: Double(position.y))
        let rho = Float(atm.p / (Double(cfg.gasConstant) * atm.t))
        let q = max(50, 0.5 * rho * speed * speed)
        let weight = cfg.aircraftMass * gravity
        drag = q * cfg.wingArea * cfg.zeroLiftDrag
            + cfg.inducedDragFactor * weight * weight / (q * cfg.wingArea)

        speed += ((thrust - drag) / cfg.aircraftMass - gravity * forward.y) * dt
        speed = max(20, min(450, speed))

        // Ground-collision avoidance overrides the stick
        let ground = Terrain.height(position.x, position.z)
        clearance = position.y - ground
        groundAvoid = projectedClearance() < Self.safeClearance

        var rollCmd = stickRoll, pitchCmd = stickPitch
        if groundAvoid {
            rollCmd = max(-1, min(1, -bank * 2.5))               // wings level
            pitchCmd = abs(bank) < 1.0 ? 1 : 0.1                 // then pull up
        }

        // Body-rate commands
        let xAxis = SIMD3<Float>(1, 0, 0), yAxis = SIMD3<Float>(0, 1, 0), zAxis = SIMD3<Float>(0, 0, 1)
        orientation = orientation
            * simd_quatf(angle: rollCmd * cfg.maxRollRate * dt, axis: xAxis)
            * simd_quatf(angle: pitchCmd * cfg.maxPitchRate * dt, axis: zAxis)
            * simd_quatf(angle: -rudder * cfg.maxYawRate * dt, axis: yAxis)

        // Coordinated turn from bank angle
        let turn = max(-0.6, min(0.6, gravity * tan(max(-1.3, min(1.3, bank))) / max(60, speed)))
        orientation = simd_quatf(angle: -turn * dt, axis: yAxis) * orientation
        orientation = simd_normalize(orientation)

        // Position, stall sink
        position += forward * speed * dt
        if speed < cfg.stallSpeed {
            position.y -= (cfg.stallSpeed - speed) / cfg.stallSpeed * 40 * dt
        }

        // Hard floor: never inside the terrain
        let floorY = Terrain.height(position.x, position.z) + Self.floorClearance
        if position.y < floorY {
            position.y = floorY
            if forward.y < 0 {
                orientation = simd_normalize(orientation * simd_quatf(angle: asin(-forward.y), axis: zAxis))
            }
        }
        position.y = min(position.y, 15_000)
    }
}

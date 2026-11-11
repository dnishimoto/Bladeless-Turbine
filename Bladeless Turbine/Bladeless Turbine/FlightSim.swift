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
    var speed: Float = 200

    /// Pilot inputs, −1…1:  roll right +, pitch up +, rudder right +.
    var stickRoll: Float = 0
    var stickPitch: Float = 0
    var rudder: Float = 0

    private(set) var groundAvoid = false
    private(set) var clearance: Float = 900
    private(set) var drag: Float = 0
    /// Speed at which the current mass-flow thrust equals drag.
    private(set) var equilibriumSpeed: Float = 0

    static let safeClearance: Float = 50
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
        speed = 200
        stickRoll = 0; stickPitch = 0; rudder = 0
        groundAvoid = false
    }

    private func projectedClearance() -> Float {
        var lowest = Float.greatestFiniteMagnitude
        let f = forward
        for t in [0.5, 1.0, 1.5] as [Float] {
            let p = position + f * (max(60, speed) * t)
            lowest = min(lowest, p.y - Terrain.height(p.x, p.z))
        }
        return lowest
    }

    /// Advance by `dt`. Speed is set by the engine's mass flow:
    ///   F(V) = ṁ_e·V_e − ṁ_air·V + (p_e − p₀)·A_e        (gross momentum − ram drag + pressure)
    ///   D(V) = q·S·C_D0 + k·W²/(q·S)
    /// The speed the jet settles at is the V where F(V) = D(V) + W·sin γ; the jet's speed
    /// follows that value with a short lag. More mass flow → higher equilibrium speed.
    func step(dt: Float, cycle: CycleResult) {
        let atm = EngineEquations.atmosphere(altitude: Double(position.y))
        let rho = Float(atm.p / (Double(cfg.gasConstant) * atm.t))
        let weight = cfg.aircraftMass * gravity
        let momentum = cycle.exhaustFlow * cycle.exhaustVelocity
        let airFlow = cycle.intakeAirFlow
        let pressureForce = cycle.pressureThrust
        let climb = weight * forward.y

        func dragAt(_ v: Float) -> Float {
            let q = max(50, 0.5 * rho * v * v)
            return q * cfg.wingArea * cfg.zeroLiftDrag
                + cfg.inducedDragFactor * weight * weight / (q * cfg.wingArea)
        }
        func excess(_ v: Float) -> Float { momentum - airFlow * v + pressureForce - dragAt(v) - climb }

        var lo: Float = 20, hi: Float = 450
        let target: Float
        if excess(lo) <= 0 { target = lo }
        else if excess(hi) >= 0 { target = hi }
        else {
            for _ in 0..<30 {
                let mid = 0.5 * (lo + hi)
                if excess(mid) > 0 { lo = mid } else { hi = mid }
            }
            target = 0.5 * (lo + hi)
        }
        equilibriumSpeed = target
        speed += (target - speed) * (1 - exp(-dt / 4))
        speed = max(20, min(450, speed))
        drag = dragAt(speed)

        // Ground-collision avoidance overrides the stick
        let ground = Terrain.height(position.x, position.z)
        clearance = position.y - ground
        // Safety net only: the pilot stays in control unless the jet is about to hit the ground,
        // and a pilot who is already pulling up is never overridden.
        let pilotPullingUp = stickPitch > 0.2
        groundAvoid = projectedClearance() < Self.safeClearance && !(pilotPullingUp && clearance > 30)

        // Stick left/right commands a bank angle (up to 60°) and a turn; released = wings level.
        let bankTarget = stickRoll * 1.05
        var rollCmd = max(-1, min(1, (bankTarget - bank) * 3)), pitchCmd = stickPitch
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
        let yawAssist: Float = groundAvoid ? 0 : stickRoll * 0.35          // rad/s extra turn authority
        orientation = simd_quatf(angle: (-turn - yawAssist) * dt, axis: yAxis) * orientation
        orientation = simd_normalize(orientation)

        // Position, stall sink
        position += forward * speed * dt
        if speed < cfg.stallSpeed {
            position.y -= (cfg.stallSpeed - speed) / cfg.stallSpeed * 15 * dt
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

//
//  FlightSim.swift
//  Bladeless Turbine
//
//  Point-mass flight model for the jet plus the procedural terrain it flies over.
//

import Foundation
import simd
import SceneKit
import UIKit

final class FlightSim {
    let cfg: TurbineConfig

    var position = SIMD3<Float>(0, 900, 0)
    var orientation = simd_quatf(angle: 0, axis: SIMD3<Float>(0, 1, 0))   // nose along +X
    var speed: Float = 70 // Initialized to match startup idle thrust equilibrium

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
        speed = 70
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

    func step(dt: Float, cycle: CycleResult) {
        let atm = EngineEquations.atmosphere(altitude: Double(position.y))
        let rho = Float(atm.p / (Double(cfg.gasConstant) * atm.t))
        let weight = cfg.aircraftMass * gravity
        let climb = weight * forward.y

        func dragAt(_ v: Float) -> Float {
            let q = max(50, 0.5 * rho * v * v)
            let zeroLift = q * cfg.wingArea * cfg.zeroLiftDrag
            let induced = cfg.inducedDragFactor * weight * weight / (q * cfg.wingArea)
            return zeroLift + induced
        }

        // Real-time Newtonian motion model (F = ma integration)
        let thrustForce = cycle.thrust
        let dragForce = dragAt(speed)
        let netForce = thrustForce - dragForce - climb
        let acceleration = netForce / cfg.aircraftMass

        speed += acceleration * dt
        speed = max(0, min(450, speed))
        drag = dragForce
        equilibriumSpeed = speed

        // Ground-collision avoidance logic removed; tracking clearance only for telemetry
        let ground = Terrain.height(position.x, position.z)
        clearance = position.y - ground
        groundAvoid = false

        // If joystick returns to center (0), target bank and pitch return to level (0)
        let bankTarget: Float = abs(stickRoll) > 0.01 ? stickRoll * 1.05 : 0.0
        let pitchTarget: Float = abs(stickPitch) > 0.01 ? stickPitch : 0.0

        let rollCmd = max(-1, min(1, (bankTarget - bank) * 3))
        let pitchCmd = pitchTarget

        let xAxis = SIMD3<Float>(1, 0, 0), yAxis = SIMD3<Float>(0, 1, 0), zAxis = SIMD3<Float>(0, 0, 1)
        orientation = orientation
            * simd_quatf(angle: rollCmd * cfg.maxRollRate * dt, axis: xAxis)
            * simd_quatf(angle: pitchCmd * cfg.maxPitchRate * dt, axis: zAxis)
            * simd_quatf(angle: -rudder * cfg.maxYawRate * dt, axis: yAxis)

        let turn = max(-0.6, min(0.6, gravity * tan(max(-1.3, min(1.3, bank))) / max(60, speed)))
        let yawAssist: Float = 0
        orientation = simd_quatf(angle: (-turn - yawAssist) * dt, axis: yAxis) * orientation
        orientation = simd_normalize(orientation)

        position += forward * speed * dt
        if speed < cfg.stallSpeed {
            position.y -= (cfg.stallSpeed - speed) / cfg.stallSpeed * 15 * dt
        }

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


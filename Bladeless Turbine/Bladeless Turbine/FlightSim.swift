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

    // The aircraft's nose points along local +X.
    // Local +Y is up and local +Z is the aircraft's right wing.
    var position = SIMD3<Float>(0, 900, 0)
    /// Starts pointing down the canyon (+Z), over the canyon floor.
    var orientation = simd_quatf(
        angle: -.pi / 2,
        axis: SIMD3<Float>(0, 1, 0)
    )
    var speed: Float = 70

    /// Pilot inputs, −1…1:
    /// roll right +, pitch up +, rudder right +.
    var stickRoll: Float = 0
    var stickPitch: Float = 0
    var rudder: Float = 0

    private(set) var groundAvoid = false
    private(set) var clearance: Float = 900
    private(set) var drag: Float = 0
    private(set) var equilibriumSpeed: Float = 0

    static let safeClearance: Float = 50
    static let floorClearance: Float = 7

    private let gravity: Float = 9.80665

    // Aircraft-local axes. These are deliberately NOT world-space vectors.
    private let localForward = SIMD3<Float>(1, 0, 0)
    private let localUp = SIMD3<Float>(0, 1, 0)
    private let localRight = SIMD3<Float>(0, 0, 1)
    private let worldUp = SIMD3<Float>(0, 1, 0)

    // MARK: - Canyon spline autopilot assistance
    // Pilot input remains active; canyon guidance adds a bounded bank request.
    private let canyonGuidanceStrength: Float = 0.85
    private let canyonLookAheadTime: Float = 2.2
    private let canyonMaximumGuidanceBank: Float = 0.48

    init(config: TurbineConfig) {
        cfg = config
    }

    var forward: SIMD3<Float> {
        orientation.act(localForward)
    }

    var up: SIMD3<Float> {
        orientation.act(localUp)
    }

    var right: SIMD3<Float> {
        orientation.act(localRight)
    }

    var altitude: Float {
        position.y
    }

    var bank: Float {
        asin(max(-1, min(1, -right.y)))
    }

    var pitchAngle: Float {
        asin(max(-1, min(1, forward.y)))
    }

    var heading: Float {
        var h = atan2(-forward.z, forward.x) * 180 / .pi
        if h < 0 {
            h += 360
        }
        return h
    }

    var mach: Float {
        let t = Float(
            EngineEquations.atmosphere(
                altitude: Double(position.y)
            ).t
        )

        return speed / (cfg.gamma * cfg.gasConstant * t).squareRoot()
    }

    func reset() {
        let startX = Terrain.canyonCenterX(z: 0)
        position = SIMD3(
            startX,
            Terrain.height(startX, 0) + 700,
            0
        )

        orientation = simd_quatf(
            angle: -.pi / 2,
            axis: localUp
        )

        speed = 70
        stickRoll = 0
        stickPitch = 0
        rudder = 0
        groundAvoid = false
    }


    /// Bank request (+ = right wing down) that steers toward the canyon axis ahead.
    private func canyonGuidanceBankTarget() -> Float {
        let lookAhead = max(140.0, min(420.0, speed * canyonLookAheadTime))
        let travelSign: Float = forward.z >= 0 ? 1 : -1          // canyon runs along Z
        let target = Terrain.canyonFlightTarget(
            aircraftX: position.x,
            aircraftZ: position.z,
            lookAhead: lookAhead * travelSign
        )

        let dx = target.x - position.x
        let dz = target.z - position.z
        let desiredHeading = atan2(-dz, dx)
        let currentForward = forward
        let currentHeading = atan2(-currentForward.z, currentForward.x)

        // Heading angle grows counter-clockwise (a LEFT turn), so a positive error needs a left bank.
        var headingError = desiredHeading - currentHeading
        while headingError > .pi { headingError -= 2.0 * .pi }
        while headingError < -.pi { headingError += 2.0 * .pi }

        let recovery = Terrain.canyonRecoveryStrength(
            x: position.x,
            z: position.z
        )
        let gain = canyonGuidanceStrength * (0.55 + recovery * 1.8)
        let leftBank = max(
            -canyonMaximumGuidanceBank,
            min(canyonMaximumGuidanceBank, headingError * gain)
        )
        return -leftBank
    }

    private func projectedClearance() -> Float {
        var lowest = Float.greatestFiniteMagnitude
        let f = forward

        for t: Float in [0.5, 1.0, 1.5] {
            let p = position + f * (max(60, speed) * t)
            lowest = min(
                lowest,
                p.y - Terrain.height(p.x, p.z)
            )
        }

        return lowest
    }

   
    func step(dt: Float, cycle: CycleResult) {

        guard dt > 0 else { return }

        let atm = EngineEquations.atmosphere(
            altitude: Double(position.y)
        )

        let rho = Float(
            atm.p / (Double(cfg.gasConstant) * atm.t)
        )

        let weight = cfg.aircraftMass * gravity
        let climb = weight * forward.y

        func dragAt(_ velocity: Float) -> Float {

            let q = max(
                50,
                0.5 * rho * velocity * velocity
            )

            let zeroLift =
                q * cfg.wingArea * cfg.zeroLiftDrag

            let induced =
                cfg.inducedDragFactor * weight * weight /
                (q * cfg.wingArea)

            return zeroLift + induced
        }

        // MARK: - Thrust and Drag

        let thrustForce = cycle.thrust
        let dragForce = dragAt(speed)

        let netForce =
            thrustForce - dragForce - climb

        let acceleration =
            netForce / cfg.aircraftMass

        speed += acceleration * dt

        speed = max(
            0,
            min(450, speed)
        )

        drag = dragForce
        equilibriumSpeed = speed

        // MARK: - Local Rotation Stack

        var rotationStack = AircraftLocalRotationStack()

        // MARK: - 1. ROLL

        // Bank angle: + = right wing down. Stick right banks right, stick left banks left.

        let rollInput =
            abs(stickRoll) > 0.01 ? stickRoll : 0

        let pilotBankTarget = rollInput * 1.05

        let canyonBankTarget = canyonGuidanceBankTarget()

        let bankTarget = max(
            -1.05,
            min(
                1.05,
                pilotBankTarget + canyonBankTarget
            )
        )

        let rollError = bankTarget - bank

        let rollCommand = max(
            -1,
            min(1, rollError * 3.0)
        )

        let rollAngle =
            rollCommand * cfg.maxRollRate * dt

        rotationStack.pushRoll(rollAngle)

        // MARK: - 2. YAW

        // Rudder right turns right (negative rotation about local up).
        let rudderInput =
            abs(rudder) > 0.01 ? rudder : 0

        let pilotYawAngle =
            -rudderInput * cfg.maxYawRate * dt

        rotationStack.pushYaw(pilotYawAngle)

        // MARK: - 3. PITCH

        // Positive pitch raises the nose.
        // Negative pitch lowers the nose.

        let pitchInput =
            abs(stickPitch) > 0.01 ? stickPitch : 0

        let pitchAngle =
            pitchInput * cfg.maxPitchRate * dt

        rotationStack.pushPitch(pitchAngle)

        // MARK: - Apply Local Rotation Stack

        // All rotations are composed in aircraft-local coordinates.
        // The order is roll, yaw, then pitch; the bank-induced turn is applied about world up.

        orientation = simd_normalize(
            orientation * rotationStack.combined
        )

        // Coordinated turn: turn rate = g·tan(bank)/V about the WORLD vertical.
        // A right bank (+) turns right, i.e. a negative rotation about +Y.
        let clampedBank = max(-1.3, min(1.3, bank))
        let coordinatedTurnRate = max(
            -0.6,
            min(0.6, gravity * tan(clampedBank) / max(60, speed))
        )
        orientation = simd_normalize(
            simd_quatf(angle: -coordinatedTurnRate * dt, axis: worldUp) * orientation
        )

        // MARK: - Forward Movement

        // Ground track advances with Mach: distance = M · a · dt, times a visual scale.
        let speedOfSound = (cfg.gamma * cfg.gasConstant * Float(atm.t)).squareRoot()
        let machNumber = speed / speedOfSound
        position += forward * (machNumber * speedOfSound * cfg.terrainSpeedScale) * dt

        // MARK: - Stall Descent

        if speed < cfg.stallSpeed {

            position.y -=
                (cfg.stallSpeed - speed) /
                cfg.stallSpeed * 15.0 * dt
        }

        // MARK: - Terrain Clearance

        let ground = Terrain.height(
            position.x,
            position.z
        )

        clearance = position.y - ground
        groundAvoid = false

        let floorY = ground + Self.floorClearance

        if position.y < floorY {

            position.y = floorY

            // Recover nose-up attitude using the aircraft's local pitch axis.

            if forward.y < 0 {

                let recoveryAngle = asin(
                    max(
                        -1,
                        min(1, -forward.y)
                    )
                )

                let recoveryRotation = simd_quatf(
                    angle: recoveryAngle,
                    axis: localRight
                )

                orientation = simd_normalize(
                    orientation * recoveryRotation
                )
            }
        }

        position.y = min(
            position.y,
            15_000
        )
    }

}




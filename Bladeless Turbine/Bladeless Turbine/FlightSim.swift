
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

    // MARK: - Aircraft Coordinate System
    //
    // Aircraft local axes:
    // +X = forward
    // +Y = up
    // +Z = right wing
    //
    // The aircraft nose points along local +X.

    var position = SIMD3<Float>(0, 900, 0)

    var orientation = simd_quatf(
        angle: 0,
        axis: SIMD3<Float>(0, 1, 0)
    )

    var speed: Float = 70

    // MARK: - Pilot Inputs
    //
    // Roll right +, pitch up +, rudder right +.
    // Input range: -1...1.

    var stickRoll: Float = 0
    var stickPitch: Float = 0
    var rudder: Float = 0

    // MARK: - Flight State

    private(set) var groundAvoid = false
    private(set) var clearance: Float = 900
    private(set) var drag: Float = 0
    private(set) var equilibriumSpeed: Float = 0

    private(set) var turbulenceLevel: Float = 0

    private var flightTime: Float = 0

    // MARK: - Aircraft Local Axes

    private let localForward = SIMD3<Float>(1, 0, 0)
    private let localUp = SIMD3<Float>(0, 1, 0)
    private let localRight = SIMD3<Float>(0, 0, 1)

    private let worldUp = SIMD3<Float>(0, 1, 0)

    // MARK: - Flight Limits

    static let safeClearance: Float = 50
    static let floorClearance: Float = 7

    // No inverted flight.
    static let bankLimit: Float = 1.45
    static let pitchLimit: Float = 1.2

    static let selfRightRate: Float = 3.0

    static let maxMach: Float = 2.0

    // MARK: - Pilot Response

    static let maxPilotBank: Float = 1.40
    static let rollResponse: Float = 7.0
    static let stickYawRate: Float = 0.45

    private let gravity: Float = 9.80665

    // MARK: - Canyon Spline Autopilot Assistance

    private let canyonGuidanceStrength: Float = 0.85
    private let canyonLookAheadTime: Float = 2.2
    private let canyonMaximumGuidanceBank: Float = 0.48

    // MARK: - Local Rotation Stack
    //
    // These are incremental rotations for the current simulation frame.
    //
    // The order is:
    // 1. Roll
    // 2. Yaw
    // 3. Pitch
    //
    // Each rotation is composed in aircraft-local coordinates.

    private struct LocalRotationStack {

        private let localForward = SIMD3<Float>(1, 0, 0)
        private let localUp = SIMD3<Float>(0, 1, 0)
        private let localRight = SIMD3<Float>(0, 0, 1)

        private var rotations: [simd_quatf] = []

        mutating func pushRoll(_ angle: Float) {
            guard abs(angle) > 0.000001 else { return }

            rotations.append(
                simd_quatf(
                    angle: angle,
                    axis: localForward
                )
            )
        }

        mutating func pushYaw(_ angle: Float) {
            guard abs(angle) > 0.000001 else { return }

            rotations.append(
                simd_quatf(
                    angle: angle,
                    axis: localUp
                )
            )
        }

        mutating func pushPitch(_ angle: Float) {
            guard abs(angle) > 0.000001 else { return }

            rotations.append(
                simd_quatf(
                    angle: angle,
                    axis: localRight
                )
            )
        }

        var combined: simd_quatf {

            var result = simd_quatf(
                angle: 0,
                axis: localUp
            )

            for rotation in rotations {
                result = result * rotation
            }

            return simd_normalize(result)
        }
    }

    // MARK: - Initialization

    init(config: TurbineConfig) {
        cfg = config
    }

    // MARK: - Aircraft Direction Vectors

    var forward: SIMD3<Float> {
        orientation.act(localForward)
    }

    var up: SIMD3<Float> {
        orientation.act(localUp)
    }

    var right: SIMD3<Float> {
        orientation.act(localRight)
    }

    // MARK: - Aircraft Measurements

    var altitude: Float {
        position.y
    }

    var bank: Float {
        asin(
            max(
                -1,
                min(1, -right.y)
            )
        )
    }

    var pitchAngle: Float {
        asin(
            max(
                -1,
                min(1, forward.y)
            )
        )
    }

    var heading: Float {

        var h = atan2(
            -forward.z,
            forward.x
        ) * 180 / .pi

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

        return speed / (
            cfg.gamma *
            cfg.gasConstant *
            t
        ).squareRoot()
    }

    // MARK: - Reset

    func reset() {

        let startZ = Terrain.canyonCenterZ(0)

        position = SIMD3(
            0,
            Terrain.height(0, startZ) + 700,
            startZ
        )

        orientation = simd_quatf(
            angle: 0,
            axis: localUp
        )

        speed = 70

        stickRoll = 0
        stickPitch = 0
        rudder = 0

        groundAvoid = false

        clearance = 700
        drag = 0
        equilibriumSpeed = 70

        turbulenceLevel = 0
        flightTime = 0
    }

    // MARK: - Canyon Outside Factor
    //
    // Returns 0 inside the flight corridor.
    // Rises toward 1 as the aircraft moves outside the canyon.

    private func canyonOutsideFactor() -> Float {

        let edge =
            Terrain.canyonPathHalfWidth -
            Terrain.canyonRecoveryMargin

        let lateral = abs(
            Terrain.canyonLateralError(
                x: position.x,
                z: position.z
            )
        )

        return max(
            0,
            min(1, (lateral - edge) / 200)
        )
    }

    // MARK: - Canyon Guidance Bank Target
    //
    // Produces a bank request that steers toward the canyon axis
    // ahead of the aircraft.

    private func canyonGuidanceBankTarget() -> Float {

        let lookAhead = max(
            140.0,
            min(
                420.0,
                speed * canyonLookAheadTime
            )
        )

        let travelSign: Float =
            forward.x >= 0 ? 1 : -1

        let target = Terrain.canyonFlightTarget(
            aircraftX: position.x,
            aircraftZ: position.z,
            lookAhead: lookAhead * travelSign
        )

        let dx = target.x - position.x
        let dz = target.z - position.z

        let desiredHeading = atan2(-dz, dx)

        let currentForward = forward

        let currentHeading = atan2(
            -currentForward.z,
            currentForward.x
        )

        // Normalize heading error.

        var headingError =
            desiredHeading - currentHeading

        while headingError > .pi {
            headingError -= 2.0 * .pi
        }

        while headingError < -.pi {
            headingError += 2.0 * .pi
        }

        let recovery = Terrain.canyonRecoveryStrength(
            x: position.x,
            z: position.z
        )

        let gain =
            canyonGuidanceStrength *
            (0.55 + recovery * 1.8)

        let outside = canyonOutsideFactor()

        let maxBank =
            canyonMaximumGuidanceBank +
            (1.1 - canyonMaximumGuidanceBank) * outside

        let leftBank = max(
            -maxBank,
            min(
                maxBank,
                headingError *
                gain *
                (1 + 1.5 * outside)
            )
        )

        return -leftBank
    }

    // MARK: - Upright Limits
    //
    // Keeps the aircraft right side up.
    // Recovers excessive bank, excessive pitch, or inverted attitude.

    private func enforceUprightLimits(dt: Float) {

        let f = forward

        let horizontal = (
            f.x * f.x +
            f.z * f.z
        ).squareRoot()

        guard horizontal > 0.02 else {
            return
        }

        let pitch = asin(
            max(
                -1,
                min(1, f.y)
            )
        )

        let levelRight = simd_normalize(
            simd_cross(f, worldUp)
        )

        let levelUp = simd_cross(
            levelRight,
            f
        )

        let r = right

        let roll = atan2(
            -simd_dot(r, levelUp),
            simd_dot(r, levelRight)
        )

        guard
            abs(roll) > Self.bankLimit ||
            abs(pitch) > Self.pitchLimit ||
            up.y < 0.05
        else {
            return
        }

        var newRoll = roll

        if abs(roll) > Self.bankLimit {

            let excess =
                abs(roll) - Self.bankLimit

            newRoll = roll -
                (roll > 0 ? 1 : -1) *
                min(
                    excess,
                    Self.selfRightRate * dt
                )
        }

        var newPitch = pitch

        if abs(pitch) > Self.pitchLimit {

            let excess =
                abs(pitch) - Self.pitchLimit

            newPitch = pitch -
                (pitch > 0 ? 1 : -1) *
                min(
                    excess,
                    Self.selfRightRate * 0.4 * dt
                )
        }

        let heading = atan2(
            -f.z,
            f.x
        )

        orientation = simd_normalize(

            simd_quatf(
                angle: heading,
                axis: worldUp
            )

            *

            simd_quatf(
                angle: newPitch,
                axis: localRight
            )

            *

            simd_quatf(
                angle: newRoll,
                axis: localForward
            )
        )
    }

    // MARK: - Projected Clearance

    private func projectedClearance() -> Float {

        var lowest = Float.greatestFiniteMagnitude

        let f = forward

        for t: Float in [0.5, 1.0, 1.5] {

            let p = position +
                f * (max(60, speed) * t)

            lowest = min(
                lowest,
                p.y - Terrain.height(p.x, p.z)
            )
        }

        return lowest
    }

    // MARK: - Main Flight Simulation Step

    func step(
        dt: Float,
        cycle: CycleResult
    ) {

        guard dt > 0 else {
            return
        }

        // MARK: - Atmosphere

        let atm = EngineEquations.atmosphere(
            altitude: Double(position.y)
        )

        let rho = Float(
            atm.p /
            (Double(cfg.gasConstant) * atm.t)
        )

        let weight = cfg.aircraftMass * gravity

        let climb = weight * forward.y

        let speedOfSound = (
            cfg.gamma *
            cfg.gasConstant *
            Float(atm.t)
        ).squareRoot()

        // MARK: - Aerodynamic Drag

        func dragAt(_ velocity: Float) -> Float {

            let q = max(
                50,
                0.5 * rho * velocity * velocity
            )

            let zeroLift =
                q *
                cfg.wingArea *
                cfg.zeroLiftDrag

            let induced =
                cfg.inducedDragFactor *
                weight * weight /
                (q * cfg.wingArea)

            // Transonic wave drag:
            // rises through Mach 0.9–1.2 and eases off supersonic.

            let m = velocity / speedOfSound

            let t = max(
                0,
                min(1, (m - 0.9) / 0.3)
            )

            let rise =
                t * t * (3 - 2 * t)

            let supersonicRelief =
                1 - 0.25 *
                max(
                    0,
                    min(1, m - 1.2)
                )

            let wave =
                q *
                cfg.wingArea *
                0.035 *
                rise *
                supersonicRelief

            return zeroLift + induced + wave
        }

        // MARK: - Thrust and Drag

        // Afterburner operates above 90% fuel command.

        let fuelCommandFraction =
            cycle.fuelAirRatio /
            max(
                1.0e-4,
                cfg.maxFuelAirRatio
            )

        let afterburner = max(
            0,
            min(
                1,
                (fuelCommandFraction - 0.9) / 0.1
            )
        )

        let thrustForce =
            cycle.thrust *
            (1 + 0.64 * afterburner)

        let dragForce = dragAt(speed)

        let netForce =
            thrustForce -
            dragForce -
            climb

        let acceleration =
            netForce / cfg.aircraftMass

        speed += acceleration * dt

        // Maximum speed: Mach 2.0.

        speed = max(
            0,
            min(
                Self.maxMach * speedOfSound,
                speed
            )
        )

        drag = dragForce
        equilibriumSpeed = speed

        // MARK: - 1. ROLL

        var rotationStack = LocalRotationStack()

        // Positive stick roll requests right bank.

        let rollInput: Float =
            abs(stickRoll) > 0.01
            ? stickRoll
            : 0

        // Outside the canyon, guidance takes over more
        // of the bank command.

        let pilotBankTarget =
            rollInput *
            Self.maxPilotBank *
            (1 - 0.85 * canyonOutsideFactor())

        let canyonBankTarget =
            canyonGuidanceBankTarget()

        let bankTarget = max(
            -Self.maxPilotBank,
            min(
                Self.maxPilotBank,
                pilotBankTarget + canyonBankTarget
            )
        )

        let rollError =
            bankTarget - bank

        let rollCommand = max(
            -1,
            min(
                1,
                rollError * Self.rollResponse
            )
        )

        let rollAngle =
            rollCommand *
            cfg.maxRollRate *
            dt

        rotationStack.pushRoll(rollAngle)

        // MARK: - 2. YAW

        // Rudder right produces a negative rotation
        // around the aircraft's local +Y axis.

        let rudderInput: Float =
            abs(rudder) > 0.01
            ? rudder
            : 0

        let pilotYawAngle =
            (
                -rudderInput * cfg.maxYawRate -
                rollInput * Self.stickYawRate
            ) * dt

        rotationStack.pushYaw(pilotYawAngle)

        // MARK: - 3. PITCH

        // Positive pitch raises the nose.
        // Negative pitch lowers the nose.

        let pitchInput: Float =
            abs(stickPitch) > 0.01
            ? stickPitch
            : 0

        let pitchAngle =
            pitchInput *
            cfg.maxPitchRate *
            dt

        rotationStack.pushPitch(pitchAngle)

        // MARK: - Apply Local Rotation Stack
        //
        // Apply roll, yaw, and pitch as one local rotation.
        // The aircraft's current orientation is the reference frame.

        orientation = simd_normalize(
            orientation * rotationStack.combined
        )

        // MARK: - Coordinated Turn
        //
        // A banked aircraft turns around world vertical.
        //
        // Positive bank = right wing down.
        // Right turn = negative rotation around world +Y.

        let clampedBank = max(
            -1.3,
            min(1.3, bank)
        )

        let coordinatedTurnRate = max(
            -0.6,
            min(
                0.6,
                gravity *
                tan(clampedBank) /
                max(60, speed)
            )
        )

        orientation = simd_normalize(
            simd_quatf(
                angle: -coordinatedTurnRate * dt,
                axis: worldUp
            ) * orientation
        )

        // MARK: - Turbulence

        flightTime += dt

        let wallProximity =
            Terrain.canyonRecoveryStrength(
                x: position.x,
                z: position.z
            )

        turbulenceLevel = max(
            0,
            min(
                1,
                0.2 +
                0.6 * wallProximity +
                0.25 * min(1, speed / 340)
            )
        )

        let t = flightTime

        // Environmental gusts.

        let gust = SIMD3<Float>(

            sin(t * 1.7) +
            0.5 * sin(t * 4.1 + 1.0),

            0.6 * sin(t * 2.3 + 2.0) +
            0.4 * sin(t * 5.9),

            sin(t * 1.3 + 0.5) +
            0.5 * sin(t * 3.7)
        )

        position +=
            gust *
            (7.0 * turbulenceLevel) *
            dt

        // Aircraft attitude shake.

        let shake = turbulenceLevel

        orientation = simd_normalize(

            orientation

            *

            simd_quatf(
                angle:
                    0.30 *
                    shake *
                    cos(t * 7.3) *
                    dt,
                axis: localForward
            )

            *

            simd_quatf(
                angle:
                    0.12 *
                    shake *
                    cos(t * 5.1 + 1.0) *
                    dt,
                axis: localRight
            )
        )

        // MARK: - Keep the Jet Right Side Up

        enforceUprightLimits(dt: dt)

        // MARK: - Forward Movement
        //
        // Movement follows the final aircraft orientation.
        // The engine and camera should use this same orientation.

        let machNumber =
            speed / speedOfSound

        position +=
            forward *
            (machNumber * speedOfSound) *
            dt

        // MARK: - Stall Descent

        if speed < cfg.stallSpeed {

            position.y -=
                (cfg.stallSpeed - speed) /
                cfg.stallSpeed *
                15.0 *
                dt
        }

        // MARK: - Terrain Clearance
        //
        // The aircraft cannot fall through the canyon floor.

        let ground = Terrain.height(
            position.x,
            position.z
        )

        clearance = position.y - ground

        groundAvoid = false

        let floorY =
            ground + Self.floorClearance

        if position.y < floorY {

            position.y = floorY

            groundAvoid = true

            // Recover nose-up attitude if the aircraft
            // is pointing downward.

            if forward.y < 0 {

                let recoveryAngle = asin(
                    max(
                        -1,
                        min(
                            1,
                            -forward.y
                        )
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

        // MARK: - Maximum Altitude

        position.y = min(
            position.y,
            15_000
        )
    }
}


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

    /// Cellular-automaton attitude recovery (takes over when control is lost).
    let recoveryCA = AttitudeRecoveryCA()
    private(set) var recoveryActive = false
    /// True while the CA is flying the jet back into the canyon (far off the guide splines or high above the rim).
    private(set) var canyonReturnActive = false
    private var recoveryLevelTime: Float = 0
    /// Seconds the CA stays out after the pilot lets go of the stick.
    private var pilotHoldoff: Float = 0

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

    // MARK: - Initialization

    init(config: TurbineConfig) {
        self.cfg = config

        position = SIMD3<Float>(0, 900, 0)

        orientation = simd_quatf(
            angle: 0,
            axis: SIMD3<Float>(0, 1, 0)
        )

        speed = 70
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
        recoveryActive = false
        canyonReturnActive = false
        pilotHoldoff = 0
        recoveryCA.reset()

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
    //
    // This is a safety correction, not a pilot control rotation.

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

    /// Full-range roll about the nose (−π…π, + = right wing down) and nose elevation.
    private func attitudeAngles() -> (roll: Float, pitch: Float) {
        let f = forward
        let pitch = asin(max(-1, min(1, f.y)))
        let horizontal = (f.x * f.x + f.z * f.z).squareRoot()
        guard horizontal > 0.02 else { return (0, pitch) }
        let levelRight = simd_normalize(simd_cross(f, worldUp))
        let levelUp = simd_cross(levelRight, f)
        let r = right
        return (atan2(-simd_dot(r, levelUp), simd_dot(r, levelRight)), pitch)
    }

    private func wrapAngle(_ a: Float) -> Float {
        var x = a
        while x > .pi { x -= 2 * .pi }
        while x < -.pi { x += 2 * .pi }
        return x
    }

    /// Decides whether control has been lost and, if so, drives roll/pitch from the CA field.
    /// Returns (rollCommand, pitchCommand) in −1…1, or nil when the pilot is in control.
    private func recoveryCommands(dt: Float) -> (roll: Float, pitch: Float)? {
        // Touching the stick hands control to the pilot immediately; the CA waits 1.5 s after release.
        if abs(stickRoll) > 0.05 || abs(stickPitch) > 0.05 {
            recoveryActive = false
            canyonReturnActive = false
            recoveryLevelTime = 0
            pilotHoldoff = 1.5
            return nil
        }
        if pilotHoldoff > 0 {
            pilotHoldoff -= dt
            return nil
        }

        let (roll, pitch) = attitudeAngles()
        let clearanceAhead = projectedClearance()
        let diving = pitch < -0.5 && clearanceAhead < 250
        let risk = max(0, min(1, 1 - clearanceAhead / 400))

        // Canyon return: far outside the guide splines or above the canyon rim, the CA's goal
        // moves from "level" to "banked toward the canyon axis, descending back into the canyon
        // if high or climbing if low".
        let centerZ = Terrain.canyonCenterZ(position.x)
        let lateral = abs(position.z - centerZ)
        let safeEdge = Terrain.canyonPathHalfWidth - Terrain.canyonRecoveryMargin
        let canyonFloor = Terrain.height(position.x, centerZ)
        let targetCanyonAltitude = canyonFloor + 180.0
        let altitudeExcess = position.y - targetCanyonAltitude

        if lateral > safeEdge + 20 || altitudeExcess > 260 {
            canyonReturnActive = true
        } else if lateral < safeEdge * 0.5 && abs(altitudeExcess) < 120 {
            canyonReturnActive = false
        }

        var goalBank: Float = 0
        var goalPitch: Float = 0.05
        if canyonReturnActive {
            // 1. Lateral homing: steer hard toward the guide path.
            let g = canyonGuidanceBankTarget() * 1.8
            goalBank = abs(g) > 0.1
                ? max(-1.15, min(1.15, g.sign == .minus ? min(g, -0.5) : max(g, 0.5)))
                : g

            // 2. Vertical homing: gravitate back down into the canyon when high, or climb when low.
            if clearanceAhead < 220 {
                goalPitch = 0.25
            } else if altitudeExcess > 40 {
                let safetyScale = max(0.0, min(1.0, (clearanceAhead - 220.0) / 250.0))
                let desiredDescent = -min(0.45, (altitudeExcess / 600.0) * 0.45)
                goalPitch = desiredDescent * safetyScale
            } else if altitudeExcess < -40 {
                goalPitch = min(0.25, (-altitudeExcess / 300.0) * 0.25)
            } else {
                goalPitch = 0.0
            }
        }

        recoveryCA.rebuildCosts(terrainRisk: risk, goalBank: goalBank, goalPitch: goalPitch)
        recoveryCA.sweep(count: 3)

        let lost = abs(roll) > Self.bankLimit * 0.95 || abs(pitch) > Self.pitchLimit * 0.95
            || up.y < 0.15 || diving
        if lost { recoveryActive = true; recoveryLevelTime = 0 }

        if canyonReturnActive && !recoveryActive {
            let t = recoveryCA.target(roll: roll, pitch: pitch)
            return (max(-1, min(1, wrapAngle(t.roll - roll) * 4)),
                    max(-1, min(1, (t.pitch - pitch) * 4)))
        }

        guard recoveryActive else { return nil }

        if abs(roll) < 0.2 && pitch > -0.1 && pitch < 0.35 && !diving {
            recoveryLevelTime += dt
            if recoveryLevelTime > 0.5 { recoveryActive = false; return nil }
        } else {
            recoveryLevelTime = 0
        }

        let target = recoveryCA.target(roll: roll, pitch: pitch)
        let rollCmd = max(-1, min(1, wrapAngle(target.roll - roll) * 4))
        let pitchCmd = max(-1, min(1, (target.pitch - pitch) * 4))
        return (rollCmd, pitchCmd)
    }

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

        // MARK: - Unified Aircraft Local Rotation Stack

        var rotationStack = AircraftLocalRotationStack()

        // MARK: - 1. ROLL

        let rollInput: Float =
            abs(stickRoll) > 0.01 ? stickRoll : 0

        let pilotBankTarget =
            rollInput *
            Self.maxPilotBank *
            (1 - 0.85 * canyonOutsideFactor())

        let canyonBankTarget = canyonGuidanceBankTarget()

        let bankTarget = max(
            -Self.maxPilotBank,
            min(
                Self.maxPilotBank,
                pilotBankTarget + canyonBankTarget
            )
        )

        let rollError = bankTarget - bank

        // When control is lost, the attitude-recovery CA replaces the pilot and the guide splines.
        let recovery = recoveryCommands(dt: dt)

        let rollCommand = recovery?.roll ?? max(
            -1,
            min(1, rollError * Self.rollResponse)
        )

        let rollAngle =
            rollCommand * cfg.maxRollRate * dt

        rotationStack.pushRoll(rollAngle)

        // MARK: - 2. YAW

        let rudderInput: Float =
            abs(rudder) > 0.01 ? rudder : 0

        let pilotYawAngle = (
            -rudderInput * cfg.maxYawRate -
            rollInput * Self.stickYawRate
        ) * dt

        rotationStack.pushYaw(pilotYawAngle)

        // MARK: - 3. PITCH

        let pitchInput: Float = recovery?.pitch ??
            (abs(stickPitch) > 0.01 ? stickPitch : 0)

        let pitchAngle =
            pitchInput * cfg.maxPitchRate * dt

        rotationStack.pushPitch(pitchAngle)

        // Predicted orientation after pilot controls.
        let pilotOrientation = simd_normalize(
            orientation * rotationStack.combined
        )

        // MARK: - 4. COORDINATED TURN

        // Calculate bank using the predicted aircraft attitude.

        let predictedRight = pilotOrientation.act(localRight)

        let predictedBank = asin(
            max(
                -1,
                min(1, -predictedRight.y)
            )
        )

        let clampedBank = max(
            -1.3,
            min(1.3, predictedBank)
        )

        // At Mach speeds g·tanφ/V is only ~0.03 rad/s, far too slow to get back into a canyon
        // 200 m wide. While the CA is flying the canyon return the jet pulls a hard, high-G turn
        // (4× the level-turn rate), so it actually changes heading in seconds.
        let turnBoost: Float = canyonReturnActive ? 4 : 1

        let coordinatedTurnRate = max(
            -0.6,
            min(
                0.6,
                turnBoost *
                gravity *
                tan(clampedBank) /
                max(60, speed)
            )
        )

        // Coordinated turn is physically around world vertical.

        let worldTurn = simd_quatf(
            angle: -coordinatedTurnRate * dt,
            axis: worldUp
        )

        // Convert the world rotation into aircraft-local coordinates.

        let localWorldTurn = simd_normalize(
            simd_conjugate(pilotOrientation) *
            worldTurn *
            pilotOrientation
        )

        rotationStack.pushRotation(localWorldTurn)

        // MARK: - 5. TURBULENCE

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

        // Turbulence attitude shake uses the shared local stack.

        let shake = turbulenceLevel

        rotationStack.pushRoll(
            0.30 *
            shake *
            cos(t * 7.3) *
            dt
        )

        rotationStack.pushPitch(
            0.12 *
            shake *
            cos(t * 5.1 + 1.0) *
            dt
        )

        // MARK: - Apply All Incremental Rotations

        orientation = simd_normalize(
            orientation * rotationStack.combined
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

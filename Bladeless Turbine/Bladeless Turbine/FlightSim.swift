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
    /// Starts pointing down the canyon (+X), over the canyon floor.
    var orientation = simd_quatf(
        angle: 0,
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
    static let maxMach: Float = 2.0

    // Left/right stick response
    static let maxPilotBank: Float = 1.40      // rad, ≈ 80° at full deflection
    static let rollResponse: Float = 7.0       // roll-rate gain on bank error (was 3)
    static let stickYawRate: Float = 0.45      // rad/s of direct turn at full stick

    private let gravity: Float = 9.80665

    /// Gust intensity 0…1: grows with Mach and with proximity to the canyon walls.
    private(set) var turbulenceLevel: Float = 0
    private var flightTime: Float = 0

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
    }


    /// Bank request (+ = right wing down) that steers toward the canyon axis ahead.
    private func canyonGuidanceBankTarget() -> Float {
        let lookAhead = max(140.0, min(420.0, speed * canyonLookAheadTime))
        let travelSign: Float = forward.x >= 0 ? 1 : -1          // canyon runs along X
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
        let speedOfSound = (cfg.gamma * cfg.gasConstant * Float(atm.t)).squareRoot()

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

            // Transonic wave drag: rises through Mach 0.9–1.2, eases off supersonic.
            let m = velocity / speedOfSound
            let t = max(0, min(1, (m - 0.9) / 0.3))
            let rise = t * t * (3 - 2 * t)
            let supersonicRelief = 1 - 0.25 * max(0, min(1, m - 1.2))
            let wave = q * cfg.wingArea * 0.035 * rise * supersonicRelief

            return zeroLift + induced + wave
        }

        // MARK: - Thrust and Drag

        // Afterburner: the last 10 % of the throttle (fuel command above 0.9) lights the burner,
        // boosting thrust up to the F100's 129.7 / 79.2 kN wet-to-dry ratio.
        let fuelCommandFraction = cycle.fuelAirRatio / max(1.0e-4, cfg.maxFuelAirRatio)
        let afterburner = max(0, min(1, (fuelCommandFraction - 0.9) / 0.1))
        let thrustForce = cycle.thrust * (1 + 0.64 * afterburner)
        let dragForce = dragAt(speed)

        let netForce =
            thrustForce - dragForce - climb

        let acceleration =
            netForce / cfg.aircraftMass

        speed += acceleration * dt

        // Top speed: Mach 2.0 at the local speed of sound.
        speed = max(
            0,
            min(Self.maxMach * speedOfSound, speed)
        )

        drag = dragForce
        equilibriumSpeed = speed

        // MARK: - Local Rotation Stack

        var rotationStack = AircraftLocalRotationStack()

        // MARK: - 1. ROLL

        // Bank angle: + = right wing down. Stick right banks right, stick left banks left.

        let rollInput =
            abs(stickRoll) > 0.01 ? stickRoll : 0

        let pilotBankTarget = rollInput * Self.maxPilotBank

        let canyonBankTarget = canyonGuidanceBankTarget()

        let bankTarget = max(
            -Self.maxPilotBank,
            min(
                Self.maxPilotBank,
                pilotBankTarget + canyonBankTarget
            )
        )

        let rollError = bankTarget - bank

        let rollCommand = max(
            -1,
            min(1, rollError * Self.rollResponse)
        )

        let rollAngle =
            rollCommand * cfg.maxRollRate * dt

        rotationStack.pushRoll(rollAngle)

        // MARK: - 2. YAW

        // Rudder right turns right (negative rotation about local up).
        let rudderInput =
            abs(rudder) > 0.01 ? rudder : 0

        let pilotYawAngle =
            (-rudderInput * cfg.maxYawRate - rollInput * Self.stickYawRate) * dt

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

        // MARK: - Turbulence

        // Gusts push the jet sideways and vertically and shake its attitude; they are strongest
        // in the narrow, fast sections near the canyon walls.
        flightTime += dt
        let wallProximity = Terrain.canyonRecoveryStrength(x: position.x, z: position.z)
        turbulenceLevel = max(0, min(1, 0.2 + 0.6 * wallProximity + 0.25 * min(1, speed / 340)))
        let t = flightTime
        let gust = SIMD3<Float>(
            sin(t * 1.7) + 0.5 * sin(t * 4.1 + 1.0),
            0.6 * sin(t * 2.3 + 2.0) + 0.4 * sin(t * 5.9),
            sin(t * 1.3 + 0.5) + 0.5 * sin(t * 3.7)
        )
        position += gust * (7.0 * turbulenceLevel) * dt
        let shake = turbulenceLevel
        orientation = simd_normalize(
            orientation
            * simd_quatf(angle: 0.30 * shake * cos(t * 7.3) * dt, axis: localForward)
            * simd_quatf(angle: 0.12 * shake * cos(t * 5.1 + 1.0) * dt, axis: localRight)
        )

        // MARK: - Forward Movement

        // Ground track at true Mach speed: distance = M · a(altitude) · dt.
        let machNumber = speed / speedOfSound
        position += forward * (machNumber * speedOfSound) * dt

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

//
//  TurbineCA.swift
//  Bladeless Turbine
//
//  Physically consistent, station-by-station thermodynamic cycle engine model
//  integrated with a Tesla disc rotor and a Cellular Automaton flow field.
//

import Foundation
import SceneKit

final class TurbineCA {
    let cfg: TurbineConfig
    private(set) var particles: [GasParticle]

    private var occupancy: [UInt8]
    private var flow: [FlowCell]
    private var nextFlow: [FlowCell]
    private var freeIDs: [Int]
    private var rng = FastRNG()

    private(set) var diskAngle: Float = 0
    private(set) var omega: Float = 1.0  // Positive idle spin (rad/s)
    private(set) var starterOn = true    // Starter active on launch
    private(set) var exhaustedTotal = 0
    private(set) var fuelUsedGrams: Float = 0
    private(set) var thrust: Float = 0
    private(set) var exhaustVelocity: Float = 0
    private(set) var exhaustMassFlow: Float = 0
    private(set) var airMassFlow: Float = 0
    private(set) var turbineTorque: Float = 0
    private(set) var compressorLoad: Float = 0
    private(set) var lpcLoad: Float = 0
    private(set) var hpcLoad: Float = 0
    private(set) var stations: [PressureStation]
    private(set) var stageCounts = [Int](repeating: 0, count: EngineStage.allCases.count)

    var fuel: Float = 0.6
    var lpcPressureRatio: Float = 2.0
    var hpcPressureRatio: Float = 2.0
    var flightMach: Float = 0.0          // Flight velocity parameter (V0 = flightMach * a0)

    static let stationNames = [
        "1. Fan Inlet",
        "2. Low-Pres Compressor",
        "3. High-Pres Compressor",
        "4. Combustion Chamber",
        "5. High-Pres Turbine",
        "6. Low-Pres Turbine",
        "7. Collector Core",
        "8. Exhaust Duct",
        "9. Rear Nozzle"
    ]

    init(config: TurbineConfig = TurbineConfig()) {
        cfg = config
        let count = config.gapCount * config.ringCount * config.sectorCount
        particles = Array(repeating: GasParticle(), count: config.maxParticles)
        occupancy = Array(repeating: 0, count: count)
        flow = Array(repeating: FlowCell(), count: count)
        nextFlow = flow
        freeIDs = Array((0..<config.maxParticles).reversed())

        stations = TurbineCA.stationNames.enumerated().map {
            PressureStation(id: $0.offset, name: $0.element, kPa: config.outletPressure / 1000.0, tempK: config.ambientTemperature)
        }
        initializeFlowField()
        starterOn = true
        omega = cfg.designOmega * 0.35 // Initial spin-up speed
    }

    func reset() {
        particles = Array(repeating: GasParticle(), count: cfg.maxParticles)
        occupancy = Array(repeating: 0, count: occupancy.count)
        freeIDs = Array((0..<cfg.maxParticles).reversed())
        diskAngle = 0
        omega = cfg.designOmega * 0.35
        starterOn = true
        exhaustedTotal = 0
        fuelUsedGrams = 0
        thrust = 0
        exhaustVelocity = 0
        exhaustMassFlow = 0
        airMassFlow = 0
        turbineTorque = 0
        compressorLoad = 0
        lpcLoad = 0
        hpcLoad = 0
        stageCounts = [Int](repeating: 0, count: EngineStage.allCases.count)
        for k in 0..<stations.count {
            stations[k].kPa = cfg.outletPressure / 1000.0
            stations[k].tempK = cfg.ambientTemperature
        }
        initializeFlowField()
    }

    func startEngine() {
        starterOn = true
        if omega < 0.5 { omega = 1.0 }
    }

    var activeCount: Int { particles.count - freeIDs.count }
    var shaftFraction: Float { omega / cfg.designOmega }
    var rpm: Float { shaftFraction * cfg.displayDesignRPM }
    var flameTemp: Float { stations[3].tempK }
    var overallPressureRatio: Float { stations[3].kPa / max(0.001, cfg.outletPressure / 1000.0) }
    var tsfc: Float { thrust > 1 ? (fuelFlowGramsPerSec / 1000.0) / (thrust / 1000.0) : 0 }
    var fuelFlowGramsPerSec: Float { fuel * cfg.maxFuelFlowGramsPerSec }

    @inline(__always)
    private func idx(_ g: Int, _ r: Int, _ t: Int) -> Int {
        (g * cfg.ringCount + r) * cfg.sectorCount + t
    }

    @inline(__always)
    private func wrapSector(_ t: Int) -> Int {
        (t % cfg.sectorCount + cfg.sectorCount) % cfg.sectorCount
    }

    func ringRadius(_ r: Int) -> Float {
        cfg.innerRingRadius + (Float(r) + 0.5) * cfg.ringWidth
    }

    func diskZ(_ i: Int) -> Float {
        (Float(i) - Float(cfg.diskCount - 1) / 2) * cfg.pitch
    }

    func gapZ(_ g: Int) -> Float {
        (Float(g) + 0.5 - Float(cfg.diskCount - 1) / 2) * cfg.pitch
    }

    func isHole(ring r: Int, sector t: Int) -> Bool {
        let rad = ringRadius(r)
        guard rad >= cfg.holeInnerRadius, rad <= cfg.holeOuterRadius else { return false }
        let theta = (Float(t) + 0.5) * cfg.sectorAngle
        let period = 2 * Float.pi / Float(cfg.holeCount)
        var d = (theta - diskAngle).truncatingRemainder(dividingBy: period)
        if d < 0 { d += period }
        if d > period / 2 { d -= period }
        return abs(d) <= cfg.holeHalfAngle
    }

    private func dynamicViscosity(_ temperature: Float) -> Float {
        let t = max(180, temperature)
        let mu = 1.716e-5 * pow(t / 273.15, 1.5) * (273.15 + 111.0) / (t + 111.0)
        return max(1.0e-5, mu)
    }

    private func initializeFlowField() {
        for g in 0..<cfg.gapCount {
            for r in 0..<cfg.ringCount {
                for t in 0..<cfg.sectorCount {
                    let k = idx(g, r, t)
                    flow[k] = FlowCell(
                        density: cfg.outletPressure / (cfg.gasConstant * cfg.ambientTemperature),
                        pressure: cfg.outletPressure,
                        temperature: cfg.ambientTemperature,
                        radialVelocity: 0,
                        tangentialVelocity: 0,
                        axialVelocity: 0,
                        totalSpecificEnergy: specificEnergy(temperature: cfg.ambientTemperature, ur: 0, ut: 0, uy: 0)
                    )
                }
            }
        }
        nextFlow = flow
        updateCycleThermodynamics(dt: 0.05)
    }

    @inline(__always)
    private func specificEnergy(temperature: Float, ur: Float, ut: Float, uy: Float) -> Float {
        let cv = cfg.gasConstant / (cfg.gamma - 1)
        return cv * temperature + 0.5 * (ur * ur + ut * ut + uy * uy)
    }

    func tick() {
        let dt = 1 / max(1, cfg.tickRate)
        diskAngle = (diskAngle + omega * dt).truncatingRemainder(dividingBy: 2 * .pi)

        // 1. Solve full thermodynamic cycle and update station properties physically
        updateCycleThermodynamics(dt: dt)

        // 2. Solve flow field sub-stepping for particle advection
        solveFlowField(dt: dt)

        // 3. Update particle tracers
        injectTracers(dt: dt)
        let start = Int(rng.next() % UInt64(max(1, particles.count)))
        for k in 0..<particles.count {
            let i = (start + k) % particles.count
            if particles[i].active { _ = advectTracer(i, dt: dt) }
        }

        // 4. Shaft Dynamics equation: Net Torque balance drives angular acceleration
        var starterTorqueVal: Float = 0
        if starterOn {
            if omega >= cfg.starterCutoff * cfg.designOmega { starterOn = false }
            else { starterTorqueVal = cfg.starterTorque * 2.5 }
        }

        let netTorque = turbineTorque - compressorLoad - (cfg.rotorDrag * omega) + starterTorqueVal
        let angularAcceleration = netTorque / max(1.0e-5, cfg.rotorInertia)
        omega = min(cfg.maxOmega, max(0.1, omega + angularAcceleration * dt))
    }

    // MARK: - Station-by-Station Thermodynamic Cycle Physics

    private func updateCycleThermodynamics(dt: Float) {
        let gamma = cfg.gamma
        let R = cfg.gasConstant
        let Cp_air = cfg.specificHeatCp
        let Cp_gas: Float = 1150.0 // Gas specific heat at elevated combustion temperatures

        // --- Station 0: Free Stream Ambient ---
        let P_amb = cfg.ambientPressure * 1000.0 // Pa
        let T_amb = cfg.ambientTemperature       // K
        let a_amb = (gamma * R * T_amb).squareRoot()
        let V0 = flightMach * a_amb

        // --- Station 1: Air Intake (Density & Mass Flow: m_dot = rho * A * V) ---
        let ductArea = Float.pi * (cfg.intakeTipRadius * cfg.intakeTipRadius - cfg.intakeHubRadius * cfg.intakeHubRadius)
        let pi_d: Float = 0.97 // Intake total pressure recovery efficiency factor
        let P01 = pi_d * P_amb * pow(1.0 + 0.5 * (gamma - 1.0) * flightMach * flightMach, gamma / (gamma - 1.0))
        let T01 = T_amb * (1.0 + 0.5 * (gamma - 1.0) * flightMach * flightMach)

        // Inlet face velocity proportional to fan spool speed
        let V1 = max(5.0, cfg.designIntakeMach * a_amb * shaftFraction)
        let rho1 = P01 / (R * T01)
        airMassFlow = rho1 * ductArea * V1 // m_dot = rho * A * V

        // --- Station 2 & 3: Low & High Pressure Compressors ---
        let pr_lpc = max(1.0, lpcPressureRatio * max(0.2, shaftFraction))
        let pr_hpc = max(1.0, hpcPressureRatio * max(0.2, shaftFraction))
        let totalPR = pr_lpc * pr_hpc

        let P02 = P01 * pr_lpc
        let T02 = T01 * (1.0 + (pow(pr_lpc, (gamma - 1.0) / gamma) - 1.0) / cfg.compressorEfficiency)

        let P03 = P01 * totalPR
        let T03 = T01 * (1.0 + (pow(totalPR, (gamma - 1.0) / gamma) - 1.0) / cfg.compressorEfficiency)

        // Compressor Mechanical Load (Power / Shaft Speed = Torque)
        let compressorPower = airMassFlow * Cp_air * (T03 - T01)
        compressorLoad = compressorPower / max(0.1, omega)
        lpcLoad = (airMassFlow * Cp_air * (T02 - T01)) / max(0.1, omega)
        hpcLoad = compressorLoad - lpcLoad

        // --- Station 4: Combustion Chamber (Full Energy Balance) ---
        let mdot_fuel = (fuel * cfg.maxFuelFlowGramsPerSec) / 1000.0 // kg/s
        fuelUsedGrams += mdot_fuel * 1000.0 * dt

        let pi_b: Float = 1.0 - cfg.burnerPressureLoss
        let P04 = P03 * pi_b

        // Enthalpy balance: T04 = (m_air * Cp_a * T03 + m_fuel * eta_b * LHV) / ((m_air + m_fuel) * Cp_g)
        let energyInAir = airMassFlow * Cp_air * T03
        let energyInFuel = mdot_fuel * cfg.combustionEfficiency * cfg.lowerHeatingValue
        let mdot_exhaust = airMassFlow + mdot_fuel
        exhaustMassFlow = mdot_exhaust

        let T04_calc = (energyInAir + energyInFuel) / max(1.0e-5, mdot_exhaust * Cp_gas)
        let T04 = min(cfg.maxFlameTemp, max(T03, T04_calc))

        // --- Station 5 & 6: Tesla Disc Turbine (Viscous Integration & Power Balance) ---
        var calculatedTorque: Float = 0
        let diskArea = Float.pi * (cfg.diskRadius * cfg.diskRadius - cfg.innerRingRadius * cfg.innerRingRadius)
        let mu = dynamicViscosity(T04)
        let shearRate = (2.0 * mu) / max(1.0e-6, cfg.effectiveFlowGap)

        // Integration across disk gap surfaces
        for _ in 0..<cfg.gapCount {
            let relativeWallVelocity = max(0.0, (cfg.maxTangentialSpeed * (T04 / T01).squareRoot()) - (omega * cfg.diskRadius))
            let gapTorque = 2.0 * shearRate * relativeWallVelocity * diskArea * (0.5 * (cfg.diskRadius + cfg.innerRingRadius))
            calculatedTorque += gapTorque
        }

        turbineTorque = calculatedTorque * cfg.turbineTorqueGain * (T04 / T01)
        let turbineWork = turbineTorque * omega
        let T05 = max(T_amb, T04 - (turbineWork / max(1.0e-5, mdot_exhaust * Cp_gas)))
        let P05 = P04 * pow(max(0.1, T05 / T04), gamma / (gamma - 1.0))

        let T06 = T05 * 0.95
        let P06 = P05 * 0.92

        // --- Station 7 & 8: Collector Core & Exhaust Duct ---
        let P07 = P06 * 0.98
        let T07 = T06

        let P08 = P07 * 0.98
        let T08 = T07 * 0.96

        // --- Station 9: Rear Nozzle & Physical Thrust Equation ---
        let nozzleRatio = max(1.0001, P08 / P_amb)
        let isentropicExpansion = 1.0 - pow(1.0 / nozzleRatio, (gamma - 1.0) / gamma)
        let Ve = (2.0 * Cp_gas * T08 * max(0.0, isentropicExpansion)).squareRoot()
        exhaustVelocity = Ve

        let Pe = P_amb // Assuming fully expanded unchoked nozzle
        let Ae = cfg.nozzleExitArea
        // F_N = (m_air + m_fuel) * Ve - m_air * V0 + (Pe - P_ambient) * Ae
        let momentumThrust = (mdot_exhaust * Ve) - (airMassFlow * V0)
        let pressureThrust = (Pe - P_amb) * Ae
        thrust = max(0.0, momentumThrust + pressureThrust)

        // --- Update Station UI Data Structures ---
        stations[0] = PressureStation(id: 0, name: TurbineCA.stationNames[0], kPa: P01 / 1000.0, tempK: T01)
        stations[1] = PressureStation(id: 1, name: TurbineCA.stationNames[1], kPa: P02 / 1000.0, tempK: T02)
        stations[2] = PressureStation(id: 2, name: TurbineCA.stationNames[2], kPa: P03 / 1000.0, tempK: T03)
        stations[3] = PressureStation(id: 3, name: TurbineCA.stationNames[3], kPa: P04 / 1000.0, tempK: T04)
        stations[4] = PressureStation(id: 4, name: TurbineCA.stationNames[4], kPa: P05 / 1000.0, tempK: T05)
        stations[5] = PressureStation(id: 5, name: TurbineCA.stationNames[5], kPa: P06 / 1000.0, tempK: T06)
        stations[6] = PressureStation(id: 6, name: TurbineCA.stationNames[6], kPa: P07 / 1000.0, tempK: T07)
        stations[7] = PressureStation(id: 7, name: TurbineCA.stationNames[7], kPa: P08 / 1000.0, tempK: T08)
        stations[8] = PressureStation(id: 8, name: TurbineCA.stationNames[8], kPa: P_amb / 1000.0, tempK: T08 * 0.85)
    }

    // MARK: - Flow Field & Particle Advection Solver

    private func solveFlowField(dt: Float) {
        let smallestCell = max(1.0e-4, min(cfg.ringWidth, cfg.innerRingRadius * cfg.sectorAngle))
        let h = max(2.0e-4, cfg.effectiveFlowGap)
        var speedBound = max(cfg.inletTangentialSpeed, omega * cfg.diskRadius)

        for g in 0..<cfg.gapCount {
            for r in 0..<cfg.ringCount {
                let rm = max(0, r - 1)
                let rp = min(cfg.ringCount - 1, r + 1)
                let span = Float(max(1, rp - rm)) * cfg.ringWidth
                for t in 0..<cfg.sectorCount {
                    let c = flow[idx(g, r, t)]
                    let dp = flow[idx(g, rp, t)].pressure - flow[idx(g, rm, t)].pressure
                    let gradient = abs(dp) / max(cfg.ringWidth, span)
                    let predictedRadial = (h * h / (12 * dynamicViscosity(c.temperature))) * gradient
                    speedBound = max(speedBound, predictedRadial, abs(c.tangentialVelocity), abs(c.radialVelocity))
                }
            }
        }
        speedBound = min(cfg.maxTangentialSpeed, max(0.5, speedBound))
        let requested = Int(ceil(dt * speedBound / max(1.0e-5, 0.45 * smallestCell)))
        let substeps = min(32, max(2, requested))
        let subDt = dt / Float(substeps)
        for _ in 0..<substeps {
            solveFlowSubstep(dt: subDt)
        }
    }

    private func solveFlowSubstep(dt: Float) {
        let dr = max(1.0e-4, cfg.ringWidth)
        let dTheta = cfg.sectorAngle
        let h = max(2.0e-4, cfg.effectiveFlowGap)
        let rIn = max(0.05, cfg.innerRingRadius)
        let rOut = max(rIn + 0.01, cfg.diskRadius)
        let logSpan = max(1.0e-5, log(rOut / rIn))

        let inletP = stations[3].kPa * 1000.0
        let outletP = stations[8].kPa * 1000.0
        let deltaP = max(0, inletP - outletP)

        let oldFlow = flow
        nextFlow = oldFlow

        for g in 0..<cfg.gapCount {
            let gapFraction = Float(g) / Float(max(1, cfg.gapCount - 1))
            let gapInletPressure = inletP - cfg.axialPressureDrop * gapFraction
            let outerRadius = rOut
            let outerTemp = stations[3].tempK
            let outerDensity = gapInletPressure / (cfg.gasConstant * max(180, outerTemp))
            let outerMu = dynamicViscosity(outerTemp)
            let outerMobility = h * h / (12 * outerMu)
            let outerGradient = deltaP / max(1.0e-5, outerRadius * logSpan)
            let outerRadialVelocity = -outerMobility * outerGradient

            for r in 0..<cfg.ringCount {
                let radius = max(0.05, ringRadius(r))
                let radialFraction = log(radius / rIn) / logSpan
                let targetPressure = outletP + deltaP * radialFraction

                for t in 0..<cfg.sectorCount {
                    let k = idx(g, r, t)
                    var c = oldFlow[k]
                    let mu = dynamicViscosity(c.temperature)
                    let rho = max(0.15, targetPressure / (cfg.gasConstant * max(180, c.temperature)))

                    let radial = outerRadialVelocity * (outerDensity / rho) * (outerRadius / radius)
                    c.radialVelocity = min(cfg.maxRadialSpeed, max(-cfg.maxRadialSpeed, radial))

                    let wallSpeed = omega * radius
                    let shearRate = (2 * mu) / max(1.0e-8, rho * h * h)
                    let relaxation = 1 - exp(-shearRate * dt)
                    c.tangentialVelocity += (wallSpeed - c.tangentialVelocity) * relaxation
                    c.tangentialVelocity = min(cfg.maxTangentialSpeed, max(-cfg.maxTangentialSpeed, c.tangentialVelocity))

                    c.pressure = targetPressure
                    c.density = rho
                    if r == cfg.ringCount - 1 {
                        c.temperature = stations[3].tempK
                        c.pressure = gapInletPressure
                        c.density = c.pressure / (cfg.gasConstant * c.temperature)
                    } else if r == 0 {
                        c.pressure = outletP
                        c.density = c.pressure / (cfg.gasConstant * c.temperature)
                    }

                    nextFlow[k] = c
                }
            }
        }
        flow = nextFlow
    }

    private func injectTracers(dt: Float) {
        let flowScale = min(3.0, max(0.4, exhaustMassFlow / max(1.0e-4, cfg.referenceMassFlow)))
        let desired = cfg.maxInjectPerTick * max(0.2, min(1, fuel)) * flowScale
        var count = Int(desired)
        if rng.unit() < desired - Float(count) { count += 1 }
        count = max(1, count)
        let outerRing = cfg.ringCount - 1

        while count > 0, let id = freeIDs.last {
            count -= 1
            let g = Int(rng.unit() * Float(cfg.gapCount)) % cfg.gapCount
            let port = Int(rng.unit() * 4) % 4
            let portCenter = port * cfg.sectorCount / 4
            let offset = Int(rng.unit() * 3) - 1
            let t = wrapSector(portCenter + offset)
            let k = idx(g, outerRing, t)
            if occupancy[k] >= cfg.cellCapacity { continue }
            freeIDs.removeLast()
            occupancy[k] += 1
            particles[id] = GasParticle(
                active: true,
                stage: .intake,
                g: g, r: outerRing, t: t,
                pg: g, pr: outerRing, pt: t,
                temp: stations[0].tempK,
                yOffset: (rng.unit() - 0.5) * 0.35,
                exhaustAge: -1,
                radialRemainder: 0.5
            )
        }
        _ = dt
    }

    @discardableResult
    private func advectTracer(_ i: Int, dt: Float) -> Float {
        var p = particles[i]
        p.pg = p.g; p.pr = p.r; p.pt = p.t

        if p.exhaustAge >= 0 {
            p.exhaustAge += 1
            p.t = wrapSector(p.t + 1)
            if p.exhaustAge > cfg.exhaustLife {
                p.active = false
                freeIDs.append(i)
            }
            particles[i] = p
            return 0
        }

        let oldK = idx(p.g, p.r, p.t)
        let cell = flow[oldK]
        let radius = max(0.05, ringRadius(p.r))
        occupancy[oldK] = occupancy[oldK] > 0 ? occupancy[oldK] - 1 : 0

        p.radialRemainder += max(0, -cell.radialVelocity) * dt / max(1.0e-5, cfg.ringWidth)
        p.angularRemainder += cell.tangentialVelocity * dt / max(1.0e-5, radius * cfg.sectorAngle)
        p.axialRemainder += max(0, cell.axialVelocity) * dt / max(1.0e-5, cfg.pitch)

        var newR = p.r
        var newT = p.t
        var newG = p.g
        if p.radialRemainder >= 1 {
            let steps = min(p.r, Int(p.radialRemainder))
            newR -= steps
            p.radialRemainder -= Float(steps)
        }
        if abs(p.angularRemainder) >= 1 {
            let steps = Int(p.angularRemainder)
            newT = wrapSector(newT + steps)
            p.angularRemainder -= Float(steps)
        }

        if p.axialRemainder >= 1, isHole(ring: newR, sector: newT) {
            if p.g + 1 < cfg.gapCount {
                newG += 1
                p.axialRemainder -= 1
            } else {
                p.exhaustAge = 0
                exhaustedTotal += 1
            }
        }

        let newK = idx(newG, newR, newT)
        if p.exhaustAge < 0 {
            if occupancy[newK] < cfg.cellCapacity {
                p.g = newG; p.r = newR; p.t = newT
                occupancy[newK] += 1
            } else {
                p.radialRemainder = min(p.radialRemainder, 0.95)
                p.angularRemainder = min(abs(p.angularRemainder), 0.95)
                occupancy[oldK] += 1
            }
        }

        let sampled = flow[idx(p.g, p.r, p.t)].temperature
        p.temp += (sampled - p.temp) * min(1, dt * 2.5)
        particles[i] = p
        return 0
    }

    func renderPosition(of p: GasParticle, alpha: Float) -> SCNVector3 {
        let nT = cfg.sectorCount
        var dT = p.t - p.pt
        if dT > nT / 2 { dT -= nT }
        else if dT < -nT / 2 { dT += nT }
        let theta = (Float(p.pt) + Float(dT) * alpha + 0.5) * cfg.sectorAngle
        let radius = ringRadius(p.pr) + (ringRadius(p.r) - ringRadius(p.pr)) * alpha

        let z: Float
        if p.exhaustAge >= 0 {
            let age = Float(p.exhaustAge) + alpha
            z = -1.35 - age * 0.18
        } else {
            let ringProgress = Float(p.r) / Float(max(1, cfg.ringCount - 1))
            z = 1.3 - (ringProgress * 1.8) + p.yOffset * 0.1
        }
        return SCNVector3(radius * cos(theta), radius * sin(theta), z)
    }

    func visualScale(of p: GasParticle) -> Float {
        let heat = max(0, min(1, (p.temp - cfg.ambientTemperature) / 1000))
        var s = 0.7 + 0.9 * heat
        if p.exhaustAge >= 0 { s *= 1 + 0.08 * Float(p.exhaustAge) }
        return s
    }
}

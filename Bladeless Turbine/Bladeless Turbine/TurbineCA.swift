//
//  TurbineCA.swift
//  Bladeless Turbine
//
//  Equation-driven, reduced-order gas-flow model for a Tesla-style disc rotor
//  integrated with longitudinal dual-spool jet engine stations, thermodynamics, and shaft mechanics.
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
    private(set) var omega: Float = 1.0  // Start with positive idle omega
    private(set) var starterOn = true   // Start enabled to spin up immediately
    private(set) var exhaustedTotal = 0
    private(set) var fuelUsedGrams: Float = 0
    private(set) var thrust: Float = 0
    private(set) var exhaustVelocity: Float = 0
    private(set) var exhaustMassFlow: Float = 0
    private(set) var turbineTorque: Float = 0
    private(set) var compressorLoad: Float = 0
    private(set) var lpcLoad: Float = 0
    private(set) var hpcLoad: Float = 0
    private(set) var stations: [PressureStation]
    private(set) var stageCounts = [Int](repeating: 0, count: EngineStage.allCases.count)

    /// Closed-form cycle (intake flow, torques, exhaust force) — see EngineEquations.swift.
    let equations: EngineEquations
    /// Engine shape shared with the SceneKit builders so tracers follow the casing.
    let layout: EngineLayout
    private(set) var cycle = CycleResult()
    /// Torque the Tesla wall-shear integral of the flow field would give (diagnostic cross-check).
    private(set) var teslaShearTorque: Float = 0

    /// Throttle position 0...1 (cutoff below 5 %).
    var fuel: Float = 0.6
    var lpcPressureRatio: Float = 2.0
    var hpcPressureRatio: Float = 2.0
    var jetIntensity: Float = 0.7

    /// Flight condition fed to the cycle (altitude in m, true airspeed in m/s).
    var altitude: Float = 0
    var airspeed: Float = 0

    /// Mass flow measured from the CA itself: tracer exits per second × mass per tracer.
    private(set) var measuredFlow: Float = 0
    private var exitEvents = 0
    private var windowTime: Float = 0

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
        equations = EngineEquations(config: config)
        layout = EngineLayout(cfg: config)
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
        omega = cfg.designOmega * 0.35 // Initial safe spin-up speed
    }

    func reset() {
        particles = Array(repeating: GasParticle(), count: cfg.maxParticles)
        occupancy = Array(repeating: 0, count: occupancy.count)
        freeIDs = Array((0..<cfg.maxParticles).reversed())
        diskAngle = 0
        omega = cfg.designOmega * 0.35
        starterOn = true
        exhaustedTotal = 0
        exitEvents = 0
        windowTime = 0
        measuredFlow = 0
        fuelUsedGrams = 0
        thrust = 0
        exhaustVelocity = 0
        exhaustMassFlow = 0
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
    }

    /// Fuel command after the throttle schedule and the overspeed governor.
    ///   below 5 % throttle : fuel cut off
    ///   otherwise          : idle fraction + (1 − idle) · throttle
    ///   governor           : trims fuel linearly over `governorBand` above 100 % design speed
    var fuelCommand: Float {
        guard fuel >= 0.05 else { return 0 }
        let scheduled = cfg.idleFuelFraction + (1 - cfg.idleFuelFraction) * fuel
        let over = max(0, shaftFraction - 1)
        return scheduled * max(0, min(1, 1 - over / max(1.0e-3, cfg.governorBand)))
    }

    var activeCount: Int { particles.count - freeIDs.count }
    var shaftFraction: Float { omega / cfg.designOmega }
    var rpm: Float { shaftFraction * cfg.displayDesignRPM }
    var flameTemp: Float { stations[3].tempK }
    var overallPressureRatio: Float { stations[3].kPa / max(1, cycle.ambientP / 1000.0) }
    var tsfc: Float { thrust > 1 ? fuelFlowGramsPerSec / (thrust / 1000) : 0 }
    var fuelFlowGramsPerSec: Float { cycle.fuelFlow * 1000 }
    var intakeAirFlow: Float { cycle.intakeAirFlow }
    var overallDesignPR: Float { max(1, lpcPressureRatio * hpcPressureRatio) }

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
            let axialFraction = Float(g) / Float(max(1, cfg.gapCount - 1))
            for r in 0..<cfg.ringCount {
                let radius = max(0.05, ringRadius(r))
                let radialFraction = log(radius / max(0.05, cfg.innerRingRadius))
                    / max(1.0e-5, log(cfg.diskRadius / max(0.05, cfg.innerRingRadius)))
                let intensity = min(1, max(0, jetIntensity))
                let combinedPR = max(1.0, lpcPressureRatio * hpcPressureRatio)
                let pBase = cfg.outletPressure
                    + ((cfg.inletPressure - cfg.outletPressure) * intensity * combinedPR
                       - cfg.axialPressureDrop * intensity * axialFraction) * radialFraction
                for t in 0..<cfg.sectorCount {
                    let temperature = cfg.ambientTemperature
                    let pressure = max(20_000, pBase)
                    let density = pressure / (cfg.gasConstant * temperature)
                    let k = idx(g, r, t)
                    flow[k] = FlowCell(
                        density: density,
                        pressure: pressure,
                        temperature: temperature,
                        radialVelocity: 0,
                        tangentialVelocity: 0,
                        axialVelocity: 0,
                        totalSpecificEnergy: specificEnergy(temperature: temperature, ur: 0, ut: 0, uy: 0)
                    )
                }
            }
        }
        nextFlow = flow
        updateStations()
    }

    @inline(__always)
    private func specificEnergy(temperature: Float, ur: Float, ut: Float, uy: Float) -> Float {
        let cv = cfg.gasConstant / (cfg.gamma - 1)
        return cv * temperature + 0.5 * (ur * ur + ut * ut + uy * uy)
    }

    func tick() {
        let dt = 1 / max(1, cfg.tickRate)
        diskAngle = (diskAngle + omega * dt).truncatingRemainder(dividingBy: 2 * .pi)

        // 1. Solve the cycle at the present shaft speed and fuel command.
        cycle = equations.solve(speedRatio: shaftFraction,
                                fuelCommand: fuelCommand,
                                overallPR: overallDesignPR,
                                altitude: altitude,
                                airspeed: airspeed)

        // 2. Flow field, tracers and stations follow the cycle.
        solveFlowField(dt: dt)
        burnFuel(dt)
        updateStations()
        updateNozzlePerformance()
        injectTracers(dt: dt)

        windowTime += dt
        if windowTime >= 1 {
            measuredFlow = Float(exitEvents) * cfg.tracerMass / windowTime
            exitEvents = 0
            windowTime = 0
        }

        let start = Int(rng.next() % UInt64(max(1, particles.count)))
        for k in 0..<particles.count {
            let i = (start + k) % particles.count
            if particles[i].active { _ = advectTracer(i, dt: dt) }
        }

        // 3. Shaft:  J · dΩ/dt = τ_turbine − τ_compressor − τ_friction + τ_starter
        var starter: Float = 0
        if starterOn {
            if shaftFraction >= cfg.starterCutoff { starterOn = false }
            else { starter = cfg.starterPhysicalTorque }
        }
        turbineTorque = cycle.turbineTorque
        compressorLoad = cycle.compressorTorque
        lpcLoad = cycle.compressorTorque * 0.4
        hpcLoad = cycle.compressorTorque * 0.6
        let netTorque = cycle.turbineTorque - cycle.compressorTorque - cycle.frictionTorque + starter
        let alpha = netTorque / max(1.0e-3, cfg.spoolInertia)                // rad/s²
        let physicalOmega = shaftFraction * cfg.physicalDesignOmega + alpha * dt
        let newFraction = max(0.05, min(cfg.maxOmega / cfg.designOmega, physicalOmega / cfg.physicalDesignOmega))
        omega = newFraction * cfg.designOmega
    }

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
        // Boundary conditions come from the cycle equations:
        //   rim (r = R)  : combustor exit  p₃, T₃
        //   centre (r→0) : turbine exit    p₅
        let burning: Float = cycle.fuelAirRatio > 0 ? 1 : 0
        let intensity = burning * min(1.2, cycle.speedRatio)
        let rimTemperature = max(180, cycle.t3)
        let outletP = cycle.p5
        let inletP = max(outletP, cycle.p3)
        let axialDrop = cfg.axialPressureDrop * intensity
        let oldFlow = flow
        nextFlow = oldFlow
        var calculatedTorque: Float = 0

        for g in 0..<cfg.gapCount {
            let gapFraction = Float(g) / Float(max(1, cfg.gapCount - 1))
            let gapInletPressure = inletP - axialDrop * gapFraction
            let deltaP = max(0, gapInletPressure - outletP)
            let outerRadius = rOut
            let outerTemp = rimTemperature
            let outerDensity = gapInletPressure / (cfg.gasConstant * outerTemp)
            let outerMu = dynamicViscosity(outerTemp)
            let outerMobility = h * h / (12 * outerMu)
            let outerGradient = deltaP / max(1.0e-5, outerRadius * logSpan)
            let outerRadialVelocity = -outerMobility * outerGradient

            for r in 0..<cfg.ringCount {
                let radius = max(0.05, ringRadius(r))
                let radialFraction = log(radius / rIn) / logSpan
                let targetPressure = outletP + deltaP * radialFraction
                let gapAxialGradient = -axialDrop
                    / max(1.0e-4, Float(max(1, cfg.gapCount - 1)) * cfg.pitch)

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
                    c.tangentialVelocity = min(cfg.maxTangentialSpeed,
                                               max(-cfg.maxTangentialSpeed, c.tangentialVelocity))

                    let axialMobility = h * h / (12 * mu)
                    let axial = -axialMobility * gapAxialGradient
                    c.axialVelocity = isHole(ring: r, sector: t)
                        ? min(2.0, max(0, axial)) : 0

                    c.pressure = targetPressure
                    c.density = rho
                    if r == cfg.ringCount - 1 {
                        c.temperature += (rimTemperature - c.temperature) * min(1, dt * 2.0)
                        c.tangentialVelocity = max(c.tangentialVelocity, cfg.inletTangentialSpeed * intensity)
                        c.pressure = gapInletPressure
                        c.density = c.pressure / (cfg.gasConstant * c.temperature)
                    } else if r == 0 {
                        c.pressure = outletP
                        c.density = c.pressure / (cfg.gasConstant * c.temperature)
                    }

                    let wallShear = 2 * mu * (c.tangentialVelocity - wallSpeed) / h
                    let faceArea = dr * radius * dTheta
                    calculatedTorque += 2 * wallShear * faceArea * radius
                    nextFlow[k] = c
                }
            }
        }

        flow = nextFlow
        teslaShearTorque = calculatedTorque * cfg.turbineTorqueGain
    }

    /// Fuel burned is the cycle's ṁ_fuel = f · ṁ_air; the heat it releases is already in T₃.
    private func burnFuel(_ dt: Float) {
        fuelUsedGrams += fuelFlowGramsPerSec * dt
    }

    // MARK: Stations and nozzle — all from the cycle equations

    private func updateStations() {
        let c = cycle
        let k = (cfg.gamma - 1) / cfg.gamma
        let etaC = cfg.compressorEfficiency, etaT = cfg.turbineEfficiency

        // Split the compressor and the expansion into two equal-ratio halves.
        let piHalfC = c.compressorPR.squareRoot()
        let tLPC = c.t1 * (1 + (pow(piHalfC, k) - 1) / etaC)
        let piHalfT = c.turbinePR.squareRoot()
        let tHPT = c.t3 * (1 - etaT * (1 - pow(piHalfT, -k)))

        let values: [(Float, Float)] = [
            (c.p1, c.t1),                       // 1. Fan inlet
            (c.p1 * piHalfC, tLPC),             // 2. Low-pressure compressor exit
            (c.p2, c.t2),                       // 3. High-pressure compressor exit
            (c.p3, c.t3),                       // 4. Combustion chamber exit / Tesla rim
            (c.p3 / piHalfT, tHPT),             // 5. First half of the Tesla expansion
            (c.p5, c.t5),                       // 6. Tesla turbine exit
            (c.p5 * 0.99, c.t5),                // 7. Collector core
            (c.p5 * 0.98, c.t5),                // 8. Exhaust duct
            (c.pe, c.te)                        // 9. Rear nozzle exit (static)
        ]
        for i in 0..<min(values.count, stations.count) {
            stations[i].kPa = values[i].0 / 1000
            stations[i].tempK = values[i].1
        }

        exhaustMassFlow = c.exhaustFlow

        // Where the gas is right now (by tracer ring), for the stage panel.
        var counts = [Int](repeating: 0, count: EngineStage.allCases.count)
        for p in particles where p.active {
            let stage: EngineStage
            if p.exhaustAge >= 0 || p.r == 0 { stage = .exhaust }
            else if p.r >= 17 { stage = .intake }
            else if p.r >= 10 { stage = .compressor }
            else if p.r >= 7 { stage = .combustor }
            else if p.r >= 3 { stage = .turbine }
            else { stage = .collector }
            counts[stage.rawValue] += 1
        }
        stageCounts = counts
    }

    /// Exhaust force from the cycle:  F = ṁ·Vₑ + (pₑ − p₀)·Aₑ  (smoothed for display).
    private func updateNozzlePerformance() {
        exhaustVelocity += 0.2 * (cycle.exhaustVelocity - exhaustVelocity)
        thrust += 0.2 * (cycle.thrust - thrust)
    }

    /// Each tracer carries `tracerMass` kg, so the injection rate is ṁ_e / m_tracer per second:
    /// the CA is fed exactly the mass flow the thermodynamic cycle produces.
    private func injectTracers(dt: Float) {
        let desired = min(60, exhaustMassFlow * dt / max(1.0e-3, cfg.tracerMass))
        var count = Int(desired)
        if rng.unit() < desired - Float(count) { count += 1 }
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
                temp: cfg.inletTemperature * 0.6,
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
        p.angularRemainder += cell.tangentialVelocity * dt
            / max(1.0e-5, radius * cfg.sectorAngle)
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
                exitEvents += 1
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

    /// Tracer position in the engine-local frame (axis +X, gas flows toward +X).
    func renderPosition(of p: GasParticle, alpha: Float) -> SCNVector3 {
        let nT = cfg.sectorCount
        var dT = p.t - p.pt
        if dT > nT / 2 { dT -= nT }
        else if dT < -nT / 2 { dT += nT }
        let theta = (Float(p.pt) + Float(dT) * alpha + 0.5) * cfg.sectorAngle
        let ring = Float(p.pr) + Float(p.r - p.pr) * alpha
        let gap = Float(p.pg) + Float(p.g - p.pg) * alpha
        let frac = max(0, min(1, 0.5 + p.yOffset / 0.35))
        let age: Float? = p.exhaustAge >= 0 ? Float(p.exhaustAge) + alpha : nil
        let q = layout.tracerPosition(ring: ring, gap: gap, angle: theta, frac: frac, exhaustAge: age)
        return SCNVector3(q.x, q.y, q.z)
    }

    func visualScale(of p: GasParticle) -> Float {
        let heat = max(0, min(1, (p.temp - cfg.ambientTemperature) / 1000))
        var s = 0.7 + 0.9 * heat
        if p.exhaustAge >= 0 { s *= 1 + 0.08 * Float(p.exhaustAge) }
        return s
    }
}

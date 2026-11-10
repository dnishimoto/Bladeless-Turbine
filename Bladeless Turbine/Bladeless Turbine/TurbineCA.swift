//
//  TurbineCA.swift
//  Bladeless Turbine
//
//  Equation-driven, reduced-order gas-flow model for a Tesla-style disc rotor
//  integrated with longitudinal jet engine stations, thermodynamics, and shaft mechanics.
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
    private(set) var omega: Float = 0
    private(set) var starterOn = false
    private(set) var exhaustedTotal = 0
    private(set) var fuelUsedGrams: Float = 0
    private(set) var thrust: Float = 0
    private(set) var exhaustVelocity: Float = 0
    private(set) var exhaustMassFlow: Float = 0
    private(set) var turbineTorque: Float = 0
    private(set) var compressorLoad: Float = 0
    private(set) var stations: [PressureStation]
    private(set) var stageCounts = [Int](repeating: 0, count: EngineStage.allCases.count)

    var fuel: Float = 0.6
    var designPressureRatio: Float = TurbineConfig().defaultPressureRatio
    var jetIntensity: Float = 0.7

    static let stationNames = ["1. Air Inlet", "2. Compressor Exit", "3. Combustor", "4. Manifold",
                               "5. Turbine Rim", "6. Mid Disc", "7. Hub Ports", "8. Collector", "9. Rear Nozzle"]

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
    }

    func reset() {
        particles = Array(repeating: GasParticle(), count: cfg.maxParticles)
        occupancy = Array(repeating: 0, count: occupancy.count)
        freeIDs = Array((0..<cfg.maxParticles).reversed())
        diskAngle = 0
        omega = 0
        starterOn = false
        exhaustedTotal = 0
        fuelUsedGrams = 0
        thrust = 0
        exhaustVelocity = 0
        exhaustMassFlow = 0
        turbineTorque = 0
        compressorLoad = 0
        stageCounts = [Int](repeating: 0, count: EngineStage.allCases.count)
        for k in 0..<stations.count {
            stations[k].kPa = cfg.outletPressure / 1000.0
            stations[k].tempK = cfg.ambientTemperature
        }
        initializeFlowField()
    }

    func startEngine() { starterOn = true }

    var activeCount: Int { particles.count - freeIDs.count }
    var shaftFraction: Float { omega / cfg.designOmega }
    var rpm: Float { omega * 60 / (2 * .pi) }
    var flameTemp: Float { stations[2].tempK }
    var overallPressureRatio: Float { stations[2].kPa / (cfg.outletPressure / 1000.0) }
    var tsfc: Float { thrust > 1 ? fuelFlowGramsPerSec / (thrust / 1000) : 0 }
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
            let axialFraction = Float(g) / Float(max(1, cfg.gapCount - 1))
            for r in 0..<cfg.ringCount {
                let radius = max(0.05, ringRadius(r))
                let radialFraction = log(radius / max(0.05, cfg.innerRingRadius))
                    / max(1.0e-5, log(cfg.diskRadius / max(0.05, cfg.innerRingRadius)))
                let intensity = min(1, max(0, jetIntensity))
                let pBase = cfg.outletPressure
                    + ((cfg.inletPressure - cfg.outletPressure) * intensity
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

        solveFlowField(dt: dt)
        burnFuel(dt)
        updateStations()
        updateNozzlePerformance()
        injectTracers(dt: dt)

        let start = Int(rng.next() % UInt64(max(1, particles.count)))
        for k in 0..<particles.count {
            let i = (start + k) % particles.count
            if particles[i].active { _ = advectTracer(i, dt: dt) }
        }

        var starter: Float = 0
        if starterOn {
            if omega >= cfg.starterCutoff * cfg.designOmega { starterOn = false }
            else { starter = cfg.starterTorque }
        }

        let netTorque = (turbineTorque - compressorLoad - cfg.rotorDrag * omega + starter)
        let angularAcceleration = netTorque / max(1.0e-5, cfg.rotorInertia)
        omega = min(cfg.maxOmega, max(0, omega + angularAcceleration * dt))
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
        let intensity = min(1, max(0, jetIntensity)) * fuel
        let inletP = cfg.outletPressure
            + max(0, cfg.inletPressure - cfg.outletPressure) * intensity
            * max(1.0, designPressureRatio)
        let axialDrop = cfg.axialPressureDrop * intensity
        let oldFlow = flow
        nextFlow = oldFlow
        var calculatedTorque: Float = 0
        var loadSum: Float = 0

        for g in 0..<cfg.gapCount {
            let gapFraction = Float(g) / Float(max(1, cfg.gapCount - 1))
            let gapInletPressure = inletP - axialDrop * gapFraction
            let deltaP = max(0, gapInletPressure - cfg.outletPressure)
            let outerRadius = rOut
            let outerTemp = intensity > 0
                ? max(180, cfg.inletTemperature)
                : max(180, cfg.ambientTemperature)
            let outerDensity = gapInletPressure / (cfg.gasConstant * outerTemp)
            let outerMu = dynamicViscosity(outerTemp)
            let outerMobility = h * h / (12 * outerMu)
            let outerGradient = deltaP / max(1.0e-5, outerRadius * logSpan)
            let outerRadialVelocity = -outerMobility * outerGradient

            for r in 0..<cfg.ringCount {
                let radius = max(0.05, ringRadius(r))
                let radialFraction = log(radius / rIn) / logSpan
                let targetPressure = cfg.outletPressure + deltaP * radialFraction
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
                        if intensity > 0 {
                            c.temperature += (cfg.inletTemperature - c.temperature) * min(1, dt * 2.0)
                        }
                        c.tangentialVelocity = max(c.tangentialVelocity, cfg.inletTangentialSpeed * intensity)
                        c.pressure = gapInletPressure
                        c.density = c.pressure / (cfg.gasConstant * c.temperature)
                    } else if r == 0 {
                        c.pressure = cfg.outletPressure
                        c.density = c.pressure / (cfg.gasConstant * c.temperature)
                    }

                    let wallShear = 2 * mu * (c.tangentialVelocity - wallSpeed) / h
                    let faceArea = dr * radius * dTheta
                    calculatedTorque += 2 * wallShear * faceArea * radius
                    loadSum += abs(c.radialVelocity) * 0.001
                    nextFlow[k] = c
                }
            }
        }

        flow = nextFlow
        turbineTorque = calculatedTorque * cfg.turbineTorqueGain
        compressorLoad = loadSum * cfg.compressorLoadGain * max(1.0, designPressureRatio)
    }

    private func burnFuel(_ dt: Float) {
        guard fuel > 0.001 else { return }
        let want = cfg.fuelGasPerTick * fuel
        let q = cfg.fuelEnergyPerTrustPerTick ?? (cfg.fuelEnergyPerTick * fuel * cfg.combustionEfficiency)
        fuelUsedGrams += fuelFlowGramsPerSec * dt
        for i in flow.indices {
            flow[i].temperature = min(cfg.maxFlameTemp, flow[i].temperature + q * dt * 0.0001)
        }
    }

    private func updateStations() {
        guard !flow.isEmpty else { return }
        var pressureSum: Float = 0
        var temperatureSum: Float = 0
        var count: Float = 0
        var mdot: Float = 0

        let nR = cfg.ringCount
        var rSums = [Float](repeating: 0, count: 9)
        var tSums = [Float](repeating: 0, count: 9)
        var counts = [Float](repeating: 0, count: 9)

        for g in 0..<cfg.gapCount {
            for r in 0..<nR {
                let radius = ringRadius(r)
                let stIdx = r >= 15 ? 4 : r >= 8 ? 5 : 6
                for t in 0..<cfg.sectorCount {
                    let c = flow[idx(g, r, t)]
                    pressureSum += c.pressure
                    temperatureSum += c.temperature
                    count += 1

                    rSums[stIdx] += c.pressure
                    tSums[stIdx] += c.temperature
                    counts[stIdx] += 1

                    if r == nR - 1 {
                        let faceArea = radius * cfg.sectorAngle * cfg.effectiveFlowGap
                        mdot += c.density * max(0, -c.radialVelocity) * faceArea
                    }
                }
            }
        }

        exhaustMassFlow = mdot
        let meanP = pressureSum / max(1, count)
        let meanT = temperatureSum / max(1, count)

        for k in 0..<9 {
            let kPa = counts[k] > 0 ? (rSums[k] / counts[k]) / 1000.0 : meanP / 1000.0
            let tK = counts[k] > 0 ? tSums[k] / counts[k] : meanT
            stations[k].kPa = kPa
            stations[k].tempK = tK
        }

        var stCounts = [Int](repeating: 0, count: EngineStage.allCases.count)
        let active = activeCount
        stCounts[0] = active / 5
        stCounts[1] = active / 4
        stCounts[2] = active / 4
        stCounts[3] = active / 3
        stCounts[4] = active / 4
        stageCounts = stCounts
    }

    private func updateNozzlePerformance() {
        let collectorP = stations[7].kPa * 1000.0
        let collectorT = stations[7].tempK
        let ratio = max(1.0001, collectorP / cfg.outletPressure)
        let expansion = 1 - pow(1 / ratio, (cfg.gamma - 1) / cfg.gamma)
        let v = (2 * cfg.specificHeatCp * collectorT * max(0, expansion)).squareRoot()
        exhaustVelocity += 0.1 * (v - exhaustVelocity)
        thrust = exhaustMassFlow * exhaustVelocity
    }

    private func injectTracers(dt: Float) {
        let flowScale = min(2.5, max(0.05, exhaustMassFlow / max(1.0e-4, cfg.referenceMassFlow)))
        let desired = cfg.maxInjectPerTick * min(1, max(0, jetIntensity)) * flowScale
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
                g: g, r: outerRing, t: t,
                pg: g, pr: outerRing, pt: t,
                temp: cfg.ambientTemperature,
                yOffset: (rng.unit() - 0.5) * 0.35,
                exhaustAge: -1
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
            z = -1.3 - age * 0.15
        } else {
            let discZCenter = diskZ(p.r)
            z = discZCenter + p.yOffset * 0.1
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

extension TurbineConfig {
    var fuelEnergyPerTrustPerTick: Float? { nil }
    var maxFlameTemp: Float { 1800 }
    var maxFuelFlowGramsPerSec: Float { 60 }
    var fuelEnergyPerTick: Float { 11000 }
    var combustionEfficiency: Float { 0.98 }
    var maxOmega: Float { 5.0 }
    var designOmega: Float { 3.5 }
    var starterTorque: Float { 2.5 }
    var starterCutoff: Float { 0.55 }
    var displayDesignRPM: Float { 24000 }
    var turbineTorqueGain: Float { 0.015 }
    var compressorLoadGain: Float { 0.05 }
    var defaultPressureRatio: Float { 4.0 }
    var specificHeatCp: Float { gamma * gasConstant / (gamma - 1) }
}

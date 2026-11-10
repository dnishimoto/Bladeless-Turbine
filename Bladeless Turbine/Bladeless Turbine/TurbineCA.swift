//
//  TurbineCA.swift
//  Bladeless Turbine
//

import Foundation
import simd

enum GasStage: UInt8 { case free, intake, turbine, collector, plume }
enum CellKind { case inlet, compressor, combustor, manifold, collector, nozzle }

struct PressureStation: Identifiable {
    let id: Int
    let name: String
    var kPa: Float
    var tempK: Float
}

final class FlowChain {
    let kind: [CellKind]
    let volume: [Float]
    let capacity: [Int]
    var occ: [Int]
    var pressure: [Float]
    var count: [Int]
    var tsum: [Float]

    init(kind: [CellKind], volume: [Float], capacity: [Int], ambient: Float) {
        self.kind = kind
        self.volume = volume
        self.capacity = capacity
        occ = Array(repeating: 0, count: kind.count)
        pressure = Array(repeating: ambient, count: kind.count)
        count = Array(repeating: 0, count: kind.count)
        tsum = Array(repeating: 0, count: kind.count)
    }

    func reset(ambient: Float) {
        for j in 0..<kind.count {
            occ[j] = 0
            pressure[j] = ambient
        }
    }
}

final class TurbineCA {
    let cfg: TurbineConfig
    private(set) var particles: [GasParticle]
    private var freeIDs: [Int]
    private var rng = FastRNG()

    private let chainA: FlowChain
    private let chainB: FlowChain
    private var occ: [Int]
    private var ringP: [Float]
    private var gapP: [Float]
    private var rimPressure: Float
    private let holeRingCount: Int

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
    private var exhaustThisTick = 0
    private var exhaustRate: Float = 0
    private var stageHeat: Float = 0

    var fuel: Float = 0.6
    var designPressureRatio: Float = TurbineConfig().defaultPressureRatio

    static let stationNames = ["1. Air Inlet", "2. Compressor Exit", "3. Combustor", "4. Manifold",
                               "5. Turbine Rim", "6. Mid Disc", "7. Hub Ports", "8. Collector", "9. Rear Nozzle"]

    static let chainAKinds: [CellKind] = [
        .inlet, .inlet,
        .compressor, .compressor, .compressor, .compressor,
        .combustor, .combustor, .combustor, .combustor, .combustor, .combustor,
        .manifold, .manifold, .manifold, .manifold
    ]
    static let chainAVolumes: [Float] = [
        2, 2,
        0.85, 0.7, 0.55, 0.45,
        0.45, 0.45, 0.45, 0.45, 0.45, 0.45,
        0.5, 0.5, 0.5, 0.5
    ]
    static let chainBKinds: [CellKind] = [
        .collector, .collector, .collector,
        .nozzle, .nozzle, .nozzle, .nozzle, .nozzle, .nozzle, .nozzle
    ]
    static let chainBVolumes: [Float] = [1.6, 1.5, 1.4, 1.3, 1.2, 1.1, 1.0, 0.9, 0.8, 0.8]

    let exhaustPath: [SIMD3<Float>]
    private let exhaustLengths: [Float]
    private let exhaustTotal: Float

    init(config: TurbineConfig = TurbineConfig()) {
        cfg = config
        let namb = config.ambientParticles
        let aCap = TurbineCA.chainAVolumes.enumerated().map { i, v in
            Int((i < 2 ? 3 : 6) * namb * v)
        }
        let bCap = TurbineCA.chainBVolumes.map { Int(4 * namb * $0) }
        chainA = FlowChain(kind: TurbineCA.chainAKinds, volume: TurbineCA.chainAVolumes,
                           capacity: aCap, ambient: config.ambientPressure)
        chainB = FlowChain(kind: TurbineCA.chainBKinds, volume: TurbineCA.chainBVolumes,
                           capacity: bCap, ambient: config.ambientPressure)
        particles = Array(repeating: GasParticle(), count: config.maxParticles)
        freeIDs = Array((0..<config.maxParticles).reversed())
        occ = Array(repeating: 0, count: config.gapCount * config.ringCount * config.sectorCount)
        ringP = Array(repeating: config.ambientPressure, count: config.ringCount)
        gapP = Array(repeating: config.ambientPressure, count: config.gapCount + 1)
        rimPressure = config.ambientPressure
        var rings = 0
        for r in 0..<config.ringCount {
            let rad = config.innerRingRadius + (Float(r) + 0.5) * config.ringWidth
            if rad <= config.holeOuterRadius { rings += 1 }
        }
        holeRingCount = max(1, rings)
        stations = TurbineCA.stationNames.enumerated().map {
            PressureStation(id: $0.offset, name: $0.element,
                            kPa: config.ambientPressure, tempK: config.ambientTemp)
        }

        // Longitudinal alignment along Z axis: Z ranges from +1.6 (Front Intake) through Z=0 (Turbine center) to -1.8 (Rear Exhaust)
        let path: [SIMD3<Float>] = [SIMD3(0, 0, 0), SIMD3(0, 0, -0.4), SIMD3(0, 0, -1.0), SIMD3(0, 0, -1.8)]
        var lens: [Float] = []
        for k in 0..<(path.count - 1) { lens.append(simd_length(path[k + 1] - path[k])) }
        exhaustPath = path
        exhaustLengths = lens
        exhaustTotal = lens.reduce(0, +)
    }

    func reset() {
        particles = Array(repeating: GasParticle(), count: cfg.maxParticles)
        freeIDs = Array((0..<cfg.maxParticles).reversed())
        chainA.reset(ambient: cfg.ambientPressure)
        chainB.reset(ambient: cfg.ambientPressure)
        occ = Array(repeating: 0, count: occ.count)
        ringP = Array(repeating: cfg.ambientPressure, count: cfg.ringCount)
        gapP = Array(repeating: cfg.ambientPressure, count: cfg.gapCount + 1)
        rimPressure = cfg.ambientPressure
        diskAngle = 0
        omega = 0
        starterOn = false
        exhaustedTotal = 0
        fuelUsedGrams = 0
        thrust = 0
        exhaustVelocity = 0
        exhaustMassFlow = 0
        exhaustThisTick = 0
        exhaustRate = 0
        stageCounts = [Int](repeating: 0, count: EngineStage.allCases.count)
        for k in 0..<stations.count {
            stations[k].kPa = cfg.ambientPressure
            stations[k].tempK = cfg.ambientTemp
        }
    }

    func startEngine() { starterOn = true }

    var activeCount: Int { particles.count - freeIDs.count }
    var shaftFraction: Float { omega / cfg.designOmega }
    var rpm: Float { shaftFraction * cfg.displayDesignRPM }
    var flameTemp: Float { stations[2].tempK }
    var overallPressureRatio: Float { stations[2].kPa / cfg.ambientPressure }
    var tsfc: Float { thrust > 1 ? fuelFlowGramsPerSec / (thrust / 1000) : 0 }
    var fuelFlowGramsPerSec: Float { fuel * cfg.maxFuelFlowGramsPerSec }

    @inline(__always)
    private func idx(_ g: Int, _ r: Int, _ t: Int) -> Int {
        (g * cfg.ringCount + r) * cfg.sectorCount + t
    }

    func ringRadius(_ r: Int) -> Float { cfg.innerRingRadius + (Float(r) + 0.5) * cfg.ringWidth }
    func diskZ(_ i: Int) -> Float { (Float(i) - Float(cfg.diskCount - 1) / 2) * cfg.pitch }
    func gapZ(_ g: Int) -> Float { (Float(g) + 0.5 - Float(cfg.diskCount - 1) / 2) * cfg.pitch }
    var turbineZCenter: Float { 0 }
    var stackHalfSpan: Float { Float(cfg.gapCount) * cfg.pitch / 2 }

    private func lerp(_ a: Float, _ b: Float, _ t: Float) -> Float { a + (b - a) * max(0, min(1, t)) }

    func ductRadius(_ c: Float) -> Float {
        if c < 2 { return 0.28 }
        if c < 6 { return lerp(0.28, 0.22, (c - 2) / 4) }
        return 0.25
    }
    func rearRadius(_ c: Float) -> Float {
        c < 3 ? lerp(0.35, 0.20, c / 3) : lerp(0.20, 0.08, (c - 3) / 7)
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

    @inline(__always)
    private func gasPressure(count n: Float, volume v: Float, ambientDensity: Float, temp: Float) -> Float {
        cfg.ambientPressure * n / max(0.0001, ambientDensity * v) * (temp / cfg.ambientTemp)
    }

    func tick() {
        let dt = 1 / cfg.tickRate
        diskAngle = (diskAngle + omega * dt).truncatingRemainder(dividingBy: 2 * .pi)

        measure(chainA, stage: .intake)
        measure(chainB, stage: .collector)
        measureTurbine()
        refillInlet()
        burnFuel(dt)

        let speedRatio = omega / cfg.designOmega
        let head = (designPressureRatio - 1) / Float(cfg.compressorStages) * speedRatio * speedRatio
        let isentropicExponent = (cfg.gamma - 1) / cfg.gamma
        stageHeat = (pow(max(1, designPressureRatio), isentropicExponent / Float(cfg.compressorStages)) - 1) * speedRatio
        var torque: Float = 0
        var load: Float = 0
        exhaustThisTick = 0

        let n = particles.count
        let start = Int(rng.next() % UInt64(n))
        for k in 0..<n {
            let i = (start + k) % n
            switch particles[i].stage {
            case .free: break
            case .intake: load += stepChain(i, chain: chainA, isFront: true, head: head)
            case .collector: _ = stepChain(i, chain: chainB, isFront: false, head: 0)
            case .turbine: torque += stepTurbine(i)
            case .plume: stepPlume(i)
            }
        }

        var starter: Float = 0
        if starterOn {
            if omega >= cfg.starterCutoff * cfg.designOmega { starterOn = false }
            else { starter = cfg.starterTorque }
        }
        turbineTorque = torque * cfg.turbineTorqueGain
        compressorLoad = load * cfg.compressorLoadGain
        let alpha = turbineTorque - compressorLoad - cfg.shaftFriction * omega + starter
        omega = max(0, min(cfg.maxOmega, omega + alpha * dt))

        updateStations()
        updateNozzlePerformance()
    }

    private func measure(_ chain: FlowChain, stage: GasStage) {
        for j in 0..<chain.kind.count { chain.count[j] = 0; chain.tsum[j] = 0 }
        for i in particles.indices where particles[i].stage == stage {
            let j = particles[i].j
            chain.count[j] += 1
            chain.tsum[j] += particles[i].temp
        }
        for j in 0..<chain.kind.count {
            let n = chain.count[j]
            let temp = n > 0 ? chain.tsum[j] / Float(n) : cfg.ambientTemp
            let raw = gasPressure(count: Float(n), volume: chain.volume[j],
                                  ambientDensity: cfg.ambientParticles, temp: temp)
            chain.pressure[j] += 0.5 * (raw - chain.pressure[j])
        }
    }

    private func measureTurbine() {
        let nR = cfg.ringCount, nG = cfg.gapCount
        var rn = [Int](repeating: 0, count: nR), rt = [Float](repeating: 0, count: nR)
        var gn = [Int](repeating: 0, count: nG), gt = [Float](repeating: 0, count: nG)
        for i in particles.indices where particles[i].stage == .turbine {
            let p = particles[i]
            rn[p.r] += 1; rt[p.r] += p.temp
            if ringRadius(p.r) <= cfg.holeOuterRadius { gn[p.g] += 1; gt[p.g] += p.temp }
        }
        let cellsPerRing = Float(cfg.sectorCount * nG)
        for r in 0..<nR {
            let temp = rn[r] > 0 ? rt[r] / Float(rn[r]) : cfg.ambientTemp
            let raw = gasPressure(count: Float(rn[r]), volume: cellsPerRing,
                                  ambientDensity: cfg.turbineAmbientPerCell, temp: temp)
            ringP[r] += 0.3 * (raw - ringP[r])
        }
        let cellsPerGapZone = Float(cfg.sectorCount * holeRingCount)
        for g in 0..<nG {
            let temp = gn[g] > 0 ? gt[g] / Float(gn[g]) : cfg.ambientTemp
            let raw = gasPressure(count: Float(gn[g]), volume: cellsPerGapZone,
                                  ambientDensity: cfg.turbineAmbientPerCell, temp: temp)
            gapP[g] += 0.3 * (raw - gapP[g])
        }
        gapP[nG] = chainB.pressure[0]
        rimPressure = ringP[nR - 1]
    }

    private func spawn(stage: GasStage, cell: Int) -> Int? {
        guard let id = freeIDs.popLast() else { return nil }
        var p = GasParticle()
        p.stage = stage
        p.j = cell
        p.temp = cfg.ambientTemp
        p.u = rng.unit() * 2 - 1
        p.v = (rng.unit() * 2 - 1) * (1 - p.u * p.u).squareRoot()
        p.jit = (rng.unit() - 0.5) * 0.7
        p.yOff = (rng.unit() - 0.5) * 0.55
        particles[id] = p
        particles[id].prev = targetPosition(of: particles[id])
        return id
    }

    private func refillInlet() {
        var budget = cfg.airRefillPerTick
        for j in [1, 0] {
            let target = Int(cfg.ambientParticles * chainA.volume[j])
            while chainA.occ[j] < target, budget > 0 {
                guard spawn(stage: .intake, cell: j) != nil else { return }
                chainA.occ[j] += 1
                budget -= 1
            }
        }
    }

    private func burnFuel(_ dt: Float) {
        guard fuel > 0.001 else { return }
        let want = cfg.fuelGasPerTick * fuel
        var n = Int(want)
        if rng.unit() < want - Float(n) { n += 1 }
        let first = 6
        while n > 0 {
            n -= 1
            if chainA.occ[first] >= chainA.capacity[first] { break }
            guard spawn(stage: .intake, cell: first) != nil else { break }
            chainA.occ[first] += 1
        }
        var ids: [Int] = []
        for i in particles.indices where particles[i].stage == .intake {
            let j = particles[i].j
            if chainA.kind[j] == .combustor { ids.append(i) }
        }
        guard !ids.isEmpty else { return }
        let q = cfg.fuelEnergyPerTick * fuel * cfg.combustionEfficiency
        let per = q / Float(ids.count)
        var used: Float = 0
        for i in ids {
            let room = cfg.maxFlameTemp - particles[i].temp
            let add = max(0, min(per, room))
            particles[i].temp += add
            used += add
        }
        let efficiency = q > 0 ? used / q : 0
        fuelUsedGrams += fuelFlowGramsPerSec * dt * efficiency
    }

    private func stepChain(_ i: Int, chain: FlowChain, isFront: Bool, head: Float) -> Float {
        var p = particles[i]
        p.prev = targetPosition(of: p)
        let n = chain.kind.count
        var j = p.j
        let startJ = j
        chain.occ[j] -= 1

        let pHere = chain.pressure[j]
        let pNext: Float = j + 1 < n ? chain.pressure[j + 1] : (isFront ? rimPressure : cfg.ambientPressure)
        let dp = (pHere - pNext) / cfg.ambientPressure
        var steps = dp > 1.2 ? 3 : dp > 0.35 ? 2 : dp > 0.03 ? 1 : 0
        if steps == 0, p.mom, dp > -0.35 { steps = 1 }
        if chain.kind[j] == .compressor, steps == 0, -dp < head { steps = 1 }

        var work: Float = 0
        var done = false
        for _ in 0..<steps {
            if j + 1 >= n {
                if isFront {
                    if enterTurbine(&p) { done = true }
                } else {
                    p.stage = .plume
                    p.age = 0
                    exhaustedTotal += 1
                    exhaustThisTick += 1
                    done = true
                }
                break
            }
            if chain.occ[j + 1] < chain.capacity[j + 1] {
                if chain.kind[j] == .compressor {
                    p.temp *= 1 + stageHeat
                    work += 1 + max(0, -dp) * 3
                }
                j += 1
            } else {
                break
            }
        }
        p.mom = (j != startJ) || done
        if !done {
            p.j = j
            chain.occ[j] += 1
        }
        if !isFront { p.temp = max(cfg.ambientTemp, p.temp * 0.985) }
        particles[i] = p
        return work
    }

    private func enterTurbine(_ p: inout GasParticle) -> Bool {
        let g = min(cfg.gapCount - 1, max(0, Int((p.v + 1) / 2 * Float(cfg.gapCount))))
        let t = Int(rng.unit() * Float(cfg.nozzleSectors)) % cfg.nozzleSectors
        let r = cfg.ringCount - 1
        let k = idx(g, r, t)
        guard occ[k] < cfg.cellCapacity else { return false }
        occ[k] += 1
        p.stage = .turbine
        p.g = g; p.r = r; p.t = t
        p.yOff = (rng.unit() - 0.5) * 0.55
        return true
    }

    private func stepTurbine(_ i: Int) -> Float {
        var p = particles[i]
        p.prev = targetPosition(of: p)
        let nT = cfg.sectorCount
        let g0 = p.g, r0 = p.r, t0 = p.t
        occ[idx(g0, r0, t0)] -= 1

        let radius = ringRadius(r0)
        let tn = min(1, p.temp / 1500)

        let stepT = rng.unit() < tn * 0.85 ? 2 : 1
        let gasOmega = Float(stepT) * cfg.sectorAngle * cfg.tickRate
        let slip = max(0, gasOmega - omega)
        let torque = slip * tn * tn * radius * 1.8
        let slipN = min(1, slip / (2 * cfg.sectorAngle * cfg.tickRate))

        var tNew = (t0 + stepT) % nT
        var rNew = r0

        if r0 > 0 {
            let pr = ringP[r0], pi = ringP[r0 - 1]
            var pin = 0.10 + 0.9 * max(0, (pr - pi) / max(pr, 0.000001)) + 0.25 * slipN
            pin = min(0.85, max(0.05, pin))
            if rng.unit() < pin, occ[idx(g0, r0 - 1, tNew)] < cfg.cellCapacity { rNew = r0 - 1 }
        }
        if occ[idx(g0, rNew, tNew)] >= cfg.cellCapacity {
            tNew = t0
            rNew = r0
        }

        var gNew = g0
        var exiting = false
        if isHole(ring: rNew, sector: tNew) {
            let pg = gapP[g0], pn = gapP[g0 + 1]
            let frac = max(0, (pg - pn) / max(pg, 0.000001))
            let pax = min(0.9, max(0.03, 0.15 + 0.75 * frac))
            if rng.unit() < pax {
                if g0 + 1 >= cfg.gapCount {
                    if chainB.occ[0] < chainB.capacity[0] { exiting = true }
                } else if occ[idx(g0 + 1, rNew, tNew)] < cfg.cellCapacity {
                    gNew = g0 + 1
                }
            }
        }

        p.temp = max(cfg.ambientTemp, p.temp * (1 - (0.006 + 0.014 * slipN)))

        if exiting {
            p.stage = .collector
            p.j = 0
            p.u = rng.unit() * 2 - 1
            p.v = (rng.unit() * 2 - 1) * (1 - p.u * p.u).squareRoot()
            p.jit = (rng.unit() - 0.5) * 0.7
            p.r = rNew; p.t = tNew
            chainB.occ[0] += 1
            particles[i] = p
            return torque
        }
        p.g = gNew; p.r = rNew; p.t = tNew
        occ[idx(gNew, rNew, tNew)] += 1
        particles[i] = p
        return torque
    }

    private func stepPlume(_ i: Int) {
        var p = particles[i]
        p.prev = targetPosition(of: p)
        p.age += 1
        p.temp = max(cfg.ambientTemp, p.temp * 0.93)
        if p.age > 16 {
            particles[i] = GasParticle()
            freeIDs.append(i)
        } else {
            particles[i] = p
        }
    }

    private func updateStations() {
        var tsum = [Float](repeating: 0, count: 9)
        var cnt = [Float](repeating: 0, count: 9)
        var stages = [Int](repeating: 0, count: EngineStage.allCases.count)
        for i in particles.indices {
            let p = particles[i]
            var z = -1
            switch p.stage {
            case .intake:
                z = p.j < 2 ? 0 : p.j < 6 ? 1 : p.j < 12 ? 2 : 3
                stages[p.j < 2 ? EngineStage.intake.rawValue
                       : p.j < 6 ? EngineStage.compression.rawValue
                       : EngineStage.combustion.rawValue] += 1
            case .turbine:
                z = p.r >= 15 ? 4 : p.r >= 8 ? 5 : 6
                stages[EngineStage.turbine.rawValue] += 1
            case .collector:
                z = p.j < 3 ? 7 : 8
                stages[EngineStage.exhaust.rawValue] += 1
            case .plume:
                stages[EngineStage.exhaust.rawValue] += 1
            case .free:
                break
            }
            if z >= 0 { tsum[z] += p.temp; cnt[z] += 1 }
        }
        stageCounts = stages
        func mean(_ a: [Float], _ r: Range<Int>) -> Float {
            r.reduce(0) { $0 + a[$1] } / Float(r.count)
        }
        let raw: [Float] = [
            mean(chainA.pressure, 0..<2),
            chainA.pressure[5],
            mean(chainA.pressure, 6..<12),
            mean(chainA.pressure, 12..<16),
            mean(ringP, 15..<cfg.ringCount),
            mean(ringP, 8..<15),
            mean(ringP, 0..<8),
            mean(chainB.pressure, 0..<3),
            mean(chainB.pressure, 3..<10)
        ]
        for k in 0..<9 {
            var target = raw[k]
            if k >= 3 { target = max(target, cfg.ambientPressure) }
            stations[k].kPa += 0.06 * (target - stations[k].kPa)
            if cnt[k] > 0 { stations[k].tempK += 0.15 * (tsum[k] / cnt[k] - stations[k].tempK) }
        }
    }

    private func updateNozzlePerformance() {
        let collector = stations[7]
        let ratio = max(1.0001, collector.kPa / cfg.ambientPressure)
        let expansion = 1 - pow(1 / ratio, (cfg.gamma - 1) / cfg.gamma)
        let v = (2 * cfg.specificHeatCp * collector.tempK * max(0, expansion)).squareRoot()
        exhaustVelocity += 0.1 * (v - exhaustVelocity)

        exhaustRate += 0.05 * (Float(exhaustThisTick) * cfg.tickRate - exhaustRate)
        exhaustMassFlow = exhaustRate * cfg.kgPerParticle
        thrust = exhaustMassFlow * exhaustVelocity
    }

    func targetPosition(of p: GasParticle) -> SIMD3<Float> {
        switch p.stage {
        case .free:
            return .zero
        case .intake:
            // Flow moves along +Z (Intake at +1.5, Compressor/Combustor leading to Turbine at Z = 0)
            let c = Float(p.j) + 0.5 + p.jit
            let totalLen = Float(cfg.chainACells) * cfg.jetCellLength
            let zPos = 1.6 - (c / Float(cfg.chainACells)) * 1.6
            let rad = ductRadius(c)
            return SIMD3(p.u * rad, p.v * rad, zPos)
        case .turbine:
            let theta = (Float(p.t) + 0.5) * cfg.sectorAngle
            let r = ringRadius(p.r)
            return SIMD3(r * cos(theta), r * sin(theta), diskZ(p.r))
        case .collector:
            let c = Float(p.j) + 0.5 + p.jit
            let s = max(0, min(1, c / Float(cfg.chainBCells))) * exhaustTotal
            var seg = 0, acc: Float = 0
            while seg < exhaustLengths.count - 1, acc + exhaustLengths[seg] < s {
                acc += exhaustLengths[seg]; seg += 1
            }
            let dir = simd_normalize(exhaustPath[seg + 1] - exhaustPath[seg])
            let center = exhaustPath[seg] + dir * (s - acc)
            let rad = rearRadius(c)
            return SIMD3(p.u * rad, p.v * rad, center.z)
        case .plume:
            let a = Float(p.age)
            let zStart = -1.8
            let zPos = zStart - Double(a) * 0.18
            let spread = 0.12 * (1.0 + 0.15 * a)
            return SIMD3(p.u * spread, p.v * spread, Float(zPos))
        }
    }

    func renderPosition(of p: GasParticle, alpha: Float) -> SIMD3<Float> {
        let cur = targetPosition(of: p)
        return p.prev + (cur - p.prev) * alpha
    }

    func visualScale(of p: GasParticle) -> Float {
        let heat = max(0, min(1, (p.temp - cfg.ambientTemp) / 1500))
        var s = 0.7 + 0.9 * heat
        if p.stage == .plume { s *= 1 + 0.08 * Float(p.age) }
        return s
    }
}


//
//  TurbineCA.swift
//  Bladeless Turbine
//
//  Cellular-automaton gas model for a bladeless (Tesla-style) turbine.
//
//  Geometry
//  --------
//  • `diskCount` disks are stacked along Y on a shared shaft. Disk 0 is a solid
//    end cap; disks 1...N-1 each have `holeCount` (3) openings, 120° apart,
//    between `holeInnerRadius` and `holeOuterRadius`. The openings are aligned
//    because all disks spin together, so they form three rotating exhaust
//    columns that gas can rise through from one gap to the next.
//  • The CA grid is (gap g) × (ring r) × (sector t) in cylindrical coordinates.
//    Gap g sits between disk g and disk g+1.
//
//  CA rules (one tick, applied to every gas particle / cell occupant)
//  ------------------------------------------------------------------
//  1. JET        A nozzle at the rim (sectors 0..<nozzleSectors, outermost ring)
//                injects hot particles (temp = 1) tangentially, in every gap.
//  2. SWIRL      A particle advances 1 or 2 sectors per tick (hot → faster).
//  3. DRAG       Slip = gas angular speed − disk angular speed. Slip both turns
//                the disks (torque) and pulls gas inward (spiral).
//  4. PRESSURE   A particle only steps inward if the inner neighbour cell is no
//                more crowded than the cell it is passing; cells have a
//                capacity, so crowding stalls flow (back-pressure).
//  5. OPENING    If the target cell lies under a rotating disk opening, the
//                particle may pass up into the next gap (and rarely back down).
//                Passing through the top disk exits into the exhaust plume.
//  6. COOLING    Temperature falls every tick, faster toward the hub.
//
//  Disk spin integrates the summed torque against friction.
//

import Foundation
import SceneKit

// MARK: - Configuration

struct TurbineConfig {
    // Disk stack
    var diskCount = 9
    var diskRadius: Float = 1.0
    var hubRadius: Float = 0.12
    var pitch: Float = 0.09
    var diskThickness: Float = 0.012

    // CA grid
    var ringCount = 20
    var sectorCount = 48            // keep a multiple of holeCount
    var innerRingRadius: Float = 0.24

    // Disk openings (three, evenly spaced)
    var holeCount = 3
    var holeInnerRadius: Float = 0.24
    var holeOuterRadius: Float = 0.52
    var holeHalfAngle: Float = 24 * .pi / 180

    // Gas
    var cellCapacity: UInt8 = 3
    var maxParticles = 1400
    var maxInjectPerTick: Float = 20
    var nozzleSectors = 2
    var tickRate: Float = 20

    var axialPassChance: Float = 0.5
    var backflowChance: Float = 0.04
    var exhaustLife = 16
    var exhaustRise: Float = 0.05

    // Disk dynamics
    var torqueGain: Float = 0.0003
    var diskFriction: Float = 0.35

    var gapCount: Int { diskCount - 1 }
    var sectorAngle: Float { 2 * .pi / Float(sectorCount) }
    var ringWidth: Float { (diskRadius - innerRingRadius) / Float(ringCount) }
}

// MARK: - Particle

struct GasParticle {
    var active = false
    var g = 0, r = 0, t = 0          // current cell
    var pg = 0, pr = 0, pt = 0       // previous cell (for render interpolation)
    var temp: Float = 1
    var yOffset: Float = 0           // vertical jitter inside the gap, in pitches
    var exhaustAge = -1              // -1 = inside the disk stack
}

// MARK: - RNG

struct FastRNG {
    var state: UInt64 = 0x9E3779B97F4A7C15
    mutating func next() -> UInt64 {
        state ^= state << 13
        state ^= state >> 7
        state ^= state << 17
        return state
    }
    /// Uniform in [0, 1)
    mutating func unit() -> Float {
        Float(next() >> 40) / Float(1 << 24)
    }
}

// MARK: - Cellular automaton

final class TurbineCA {
    let cfg: TurbineConfig
    private(set) var particles: [GasParticle]
    private var occupancy: [UInt8]
    private var freeIDs: [Int]
    private var rng = FastRNG()

    private(set) var diskAngle: Float = 0      // rad, CA angle of opening #0
    private(set) var diskOmega: Float = 0      // rad/s
    private(set) var exhaustedTotal = 0

    /// 0...1 – scales how many particles the nozzle injects per tick.
    var jetIntensity: Float = 0.7

    init(config: TurbineConfig = TurbineConfig()) {
        cfg = config
        particles = Array(repeating: GasParticle(), count: config.maxParticles)
        occupancy = Array(repeating: 0,
                          count: config.gapCount * config.ringCount * config.sectorCount)
        freeIDs = Array((0..<config.maxParticles).reversed())
    }

    func reset() {
        particles = Array(repeating: GasParticle(), count: cfg.maxParticles)
        occupancy = Array(repeating: 0, count: occupancy.count)
        freeIDs = Array((0..<cfg.maxParticles).reversed())
        diskAngle = 0
        diskOmega = 0
        exhaustedTotal = 0
    }

    var activeCount: Int { particles.count - freeIDs.count }
    var rpm: Float { diskOmega * 60 / (2 * .pi) }

    // MARK: Geometry helpers

    @inline(__always)
    private func idx(_ g: Int, _ r: Int, _ t: Int) -> Int {
        (g * cfg.ringCount + r) * cfg.sectorCount + t
    }

    func ringRadius(_ r: Int) -> Float {
        cfg.innerRingRadius + (Float(r) + 0.5) * cfg.ringWidth
    }

    func diskY(_ i: Int) -> Float {
        (Float(i) - Float(cfg.diskCount - 1) / 2) * cfg.pitch
    }

    func gapY(_ g: Int) -> Float {
        (Float(g) + 0.5 - Float(cfg.diskCount - 1) / 2) * cfg.pitch
    }

    /// True when the (ring, sector) column currently lies under a disk opening.
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

    // MARK: Tick

    func tick() {
        let dt = 1 / cfg.tickRate
        diskAngle = (diskAngle + diskOmega * dt).truncatingRemainder(dividingBy: 2 * .pi)

        inject()

        var torque: Float = 0
        let n = particles.count
        let start = Int(rng.next() % UInt64(n))      // random sweep start avoids directional bias
        for k in 0..<n {
            let i = (start + k) % n
            if particles[i].active { torque += step(i) }
        }

        let alpha = torque * cfg.torqueGain - cfg.diskFriction * diskOmega
        diskOmega = max(0, diskOmega + alpha * dt)
    }

    // MARK: Injection (the jet)

    private func inject() {
        let want = cfg.maxInjectPerTick * jetIntensity
        var n = Int(want)
        if rng.unit() < want - Float(n) { n += 1 }
        let ring = cfg.ringCount - 1

        while n > 0, let id = freeIDs.last {
            n -= 1
            let g = Int(rng.unit() * Float(cfg.gapCount)) % cfg.gapCount
            let t = Int(rng.unit() * Float(cfg.nozzleSectors)) % cfg.nozzleSectors
            let k = idx(g, ring, t)
            if occupancy[k] >= cfg.cellCapacity { continue }
            freeIDs.removeLast()
            occupancy[k] += 1
            particles[id] = GasParticle(active: true,
                                        g: g, r: ring, t: t,
                                        pg: g, pr: ring, pt: t,
                                        temp: 1,
                                        yOffset: (rng.unit() - 0.5) * 0.55,
                                        exhaustAge: -1)
        }
    }

    // MARK: Per-particle rule. Returns torque contributed to the disks.

    private func step(_ i: Int) -> Float {
        var p = particles[i]
        let nT = cfg.sectorCount
        let nR = cfg.ringCount
        p.pg = p.g; p.pr = p.r; p.pt = p.t

        // Exhaust plume: leaves the stack, spirals up, then is recycled.
        if p.exhaustAge >= 0 {
            p.exhaustAge += 1
            p.t = (p.t + 2) % nT
            if p.exhaustAge > cfg.exhaustLife {
                p.active = false
                freeIDs.append(i)
            }
            particles[i] = p
            return 0
        }

        let g0 = p.g, r0 = p.r, t0 = p.t
        occupancy[idx(g0, r0, t0)] -= 1

        let radius = ringRadius(r0)
        let rFrac = (radius - cfg.innerRingRadius) / (cfg.diskRadius - cfg.innerRingRadius)

        // 2. SWIRL
        let stepT = rng.unit() < p.temp * 0.85 ? 2 : 1
        let gasOmega = Float(stepT) * cfg.sectorAngle * cfg.tickRate

        // 3. DRAG / torque
        let slip = max(0, gasOmega - diskOmega)
        let torque = slip * p.temp * radius
        let slipNorm = min(1, slip / (2 * cfg.sectorAngle * cfg.tickRate))

        var tNew = (t0 + stepT) % nT
        var rNew = r0

        // 4. PRESSURE-driven inward spiral
        let pIn = 0.22 + 0.45 * slipNorm
        if r0 > 0, rng.unit() < pIn {
            let here = occupancy[idx(g0, r0, tNew)]
            let inner = occupancy[idx(g0, r0 - 1, tNew)]
            if inner < cfg.cellCapacity, inner <= here { rNew = r0 - 1 }
        }

        // Congestion: blocked target → stall in place
        if occupancy[idx(g0, rNew, tNew)] >= cfg.cellCapacity {
            tNew = t0
            rNew = r0
        }

        // 5. OPENINGS
        var gNew = g0
        var exiting = false
        if isHole(ring: rNew, sector: tNew) {
            let u = rng.unit()
            if u < cfg.axialPassChance {
                if g0 + 1 >= cfg.gapCount {
                    exiting = true
                } else if occupancy[idx(g0 + 1, rNew, tNew)] < cfg.cellCapacity {
                    gNew = g0 + 1
                }
            } else if u > 1 - cfg.backflowChance, g0 > 0,
                      occupancy[idx(g0 - 1, rNew, tNew)] < cfg.cellCapacity {
                gNew = g0 - 1
            }
        }

        // 6. COOLING
        p.temp = max(0.12, p.temp - (0.003 + 0.010 * (1 - rFrac)))

        if exiting {
            p.r = rNew; p.t = tNew; p.g = g0
            p.exhaustAge = 0
            exhaustedTotal += 1
            particles[i] = p
            return torque
        }

        p.g = gNew; p.r = rNew; p.t = tNew
        occupancy[idx(gNew, rNew, tNew)] += 1
        particles[i] = p
        return torque
    }

    // MARK: Render position (interpolated between ticks, alpha in 0...1)

    func position(of p: GasParticle, alpha: Float) -> SCNVector3 {
        let nT = cfg.sectorCount
        var dT = p.t - p.pt
        if dT > nT / 2 { dT -= nT } else if dT < -nT / 2 { dT += nT }
        let theta = (Float(p.pt) + Float(dT) * alpha + 0.5) * cfg.sectorAngle
        var radius = ringRadius(p.pr) + (ringRadius(p.r) - ringRadius(p.pr)) * alpha

        let y: Float
        if p.exhaustAge >= 0 {
            let a = Float(p.exhaustAge) + alpha
            y = gapY(cfg.gapCount - 1) + a * cfg.exhaustRise + p.yOffset * cfg.pitch
            radius *= 1 + 0.025 * a
        } else {
            y = gapY(p.pg) + (gapY(p.g) - gapY(p.pg)) * alpha + p.yOffset * cfg.pitch
        }
        return SCNVector3(radius * cos(theta), y, -radius * sin(theta))
    }
}

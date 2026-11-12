//
//  AttitudeRecoveryCA.swift
//  Bladeless Turbine
//
//  A cellular automaton that finds the way back to level flight.
//
//  The CA grid is attitude space: columns = bank angle (−180…180°, wraps around),
//  rows = pitch angle (−90…90°). Every cell holds its "distance to safe flight".
//
//  Rule, applied to every cell each sweep (Moore neighbourhood, 8 neighbours):
//      goal cell (level, slightly nose up)  →  0
//      otherwise                            →  cost(cell) + min(neighbour values)
//
//  Costs make some attitudes expensive to pass through: inverted flight, extreme pitch,
//  and — when the ground is close — nose-down attitudes. The field therefore relaxes into
//  a landscape whose steepest descent is the cheapest recovery path; the controller simply
//  steps to the lowest-valued neighbour of the jet's current cell.
//

import Foundation

final class AttitudeRecoveryCA {
    static let bankCells = 36          // 10° per cell
    static let pitchCells = 19         // 10° per cell
    private let cellSize: Float = .pi / 18

    private(set) var field: [Float]
    private var cost: [Float]
    private var goal: [Bool]
    private var scratch: [Float]
    private(set) var currentCell = (bank: 18, pitch: 9)
    private(set) var targetCell = (bank: 18, pitch: 9)
    /// Attitude the goal region is centred on (level by default; banked toward the canyon when returning).
    private(set) var goalBank: Float = 0
    private(set) var goalPitch: Float = 0.1

    init() {
        let n = Self.bankCells * Self.pitchCells
        field = Array(repeating: 1000, count: n)
        scratch = field
        cost = Array(repeating: 1, count: n)
        goal = Array(repeating: false, count: n)
        rebuildCosts(terrainRisk: 0)
    }

    // MARK: Grid <-> angle helpers

    private func index(_ b: Int, _ p: Int) -> Int { p * Self.bankCells + ((b % Self.bankCells) + Self.bankCells) % Self.bankCells }

    /// Bank angle (rad) at the centre of column b.
    func bankAngle(_ b: Int) -> Float { -.pi + (Float(b) + 0.5) * cellSize }
    /// Pitch angle (rad) at the centre of row p.
    func pitchAngle(_ p: Int) -> Float { -.pi / 2 + (Float(p) + 0.5) * cellSize }

    private func bankColumn(_ bank: Float) -> Int {
        var b = Int(floor((bank + .pi) / cellSize))
        b = ((b % Self.bankCells) + Self.bankCells) % Self.bankCells
        return b
    }
    private func pitchRow(_ pitch: Float) -> Int {
        max(0, min(Self.pitchCells - 1, Int(floor((pitch + .pi / 2) / cellSize))))
    }

    // MARK: Costs

    /// terrainRisk 0…1: how urgent it is to avoid nose-down attitudes (ground close ahead).
    func rebuildCosts(terrainRisk: Float, goalBank gb: Float = 0, goalPitch gp: Float = 0.1) {
        goalBank = gb
        goalPitch = gp
        for p in 0..<Self.pitchCells {
            let pitch = pitchAngle(p)
            for b in 0..<Self.bankCells {
                let bank = bankAngle(b)
                var c: Float = 1
                if abs(bank) > .pi / 2 { c += 0.6 }                         // inverted
                if abs(pitch) > 1.2 { c += 2.0 }                           // near vertical
                if pitch < -0.1 { c += terrainRisk * 6 * min(1, -pitch / 0.6) }   // diving toward the ground
                if abs(bank) > 1.2 { c += 0.3 }                            // steep bank
                let k = index(b, p)
                cost[k] = c
                // Goal: wings level (±10°), pitch 0…+10° (slightly nose up).
                goal[k] = abs(wrapPi(bank - gb)) < 0.18 && pitch >= gp - 0.15 && pitch <= gp + 0.2
            }
        }
    }

    // MARK: CA sweeps

    func sweep(count: Int) {
        for _ in 0..<count {
            for p in 0..<Self.pitchCells {
                for b in 0..<Self.bankCells {
                    let k = index(b, p)
                    if goal[k] { scratch[k] = 0; continue }
                    var best: Float = .greatestFiniteMagnitude
                    for dp in -1...1 {
                        let np = p + dp
                        if np < 0 || np >= Self.pitchCells { continue }
                        for db in -1...1 where !(db == 0 && dp == 0) {
                            best = min(best, field[index(b + db, np)])
                        }
                    }
                    scratch[k] = min(1000, cost[k] + best)
                }
            }
            swap(&field, &scratch)
        }
    }

    private func wrapPi(_ a: Float) -> Float {
        var x = a
        while x > .pi { x -= 2 * .pi }
        while x < -.pi { x += 2 * .pi }
        return x
    }

    // MARK: Control

    /// Where the CA says to go from the current attitude: target (bank, pitch) in radians.
    /// `roll` is the full-range roll about the nose (−π…π), `pitch` the nose elevation.
    func target(roll: Float, pitch: Float) -> (roll: Float, pitch: Float) {
        let b = bankColumn(roll), p = pitchRow(pitch)
        currentCell = (b, p)
        var best = field[index(b, p)]
        var bb = b, bp = p
        for dp in -1...1 {
            let np = p + dp
            if np < 0 || np >= Self.pitchCells { continue }
            for db in -1...1 where !(db == 0 && dp == 0) {
                let v = field[index(b + db, np)]
                if v < best { best = v; bb = b + db; bp = np }
            }
        }
        targetCell = (((bb % Self.bankCells) + Self.bankCells) % Self.bankCells, bp)
        if best <= 0 { return (goalBank, max(goalPitch, pitch)) }   // already in the goal region
        return (bankAngle(bb), pitchAngle(bp))
    }

    func reset() {
        for i in 0..<field.count { field[i] = 1000 }
    }
}

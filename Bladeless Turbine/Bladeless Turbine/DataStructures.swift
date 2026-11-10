//
//  DataStructures.swift
//  Bladeless Turbine
//

import Foundation
import SwiftUI
import SceneKit

enum EngineStage: Int, CaseIterable, Identifiable {
    case intake, compression, combustion, turbine, exhaust

    var id: Int { rawValue }

    var title: String {
        switch self {
        case .intake: return "1. INTAKE"
        case .compression: return "2. COMPRESSOR"
        case .combustion: return "3. COMBUSTOR"
        case .turbine: return "4. TESLA TURBINE"
        case .exhaust: return "5. EXHAUST NOZZLE"
        }
    }

    var color: UIColor {
        switch self {
        case .intake: return .cyan
        case .compression: return .blue
        case .combustion: return .orange
        case .turbine: return .yellow
        case .exhaust: return .red
        }
    }
}

struct TurbineConfig {
    var diskCount = 9
    var diskRadius: Float = 1.0
    var hubRadius: Float = 0.12
    var pitch: Float = 0.09
    var diskThickness: Float = 0.012

    var ringCount = 20
    var sectorCount = 48
    var innerRingRadius: Float = 0.24
    var holeCount = 3
    var holeInnerRadius: Float = 0.24
    var holeOuterRadius: Float = 0.52
    var holeHalfAngle: Float = 24 * .pi / 180
    var cellCapacity = 3
    var nozzleSectors = 3
    var turbineAmbientPerCell: Float = 0.15

    var ambientParticles: Float = 6
    var ambientPressure: Float = 101.325
    var ambientTemp: Float = 300
    var tickRate: Float = 20
    var maxParticles = 2400
    var airRefillPerTick = 60

    var gamma: Float = 1.4
    var gasConstant: Float = 287.05
    var kgPerParticle: Float = 0.01

    var compressorStages = 4
    var defaultPressureRatio: Float = 4
    var designOmega: Float = 3.5
    var maxOmega: Float = 5
    var compressorLoadGain: Float = 0.05

    var fuelEnergyPerTick: Float = 11000
    var fuelGasPerTick: Float = 5
    var combustionEfficiency: Float = 0.98
    var maxFlameTemp: Float = 2000
    var maxFuelFlowGramsPerSec: Float = 60

    var turbineTorqueGain: Float = 0.010
    var shaftFriction: Float = 0.35
    var starterTorque: Float = 2.5
    var starterCutoff: Float = 0.55
    var displayDesignRPM: Float = 24000

    var chainACells = 16
    var chainBCells = 10
    var jetCellLength: Float = 0.12

    var gapCount: Int { diskCount - 1 }
    var sectorAngle: Float { 2 * .pi / Float(sectorCount) }
    var ringWidth: Float { (diskRadius - innerRingRadius) / Float(ringCount) }
    var specificHeatCp: Float { gamma * gasConstant / (gamma - 1) }
}

struct GasParticle {
    var stage: GasStage = .free
    var j = 0
    var g = 0, r = 0, t = 0
    var temp: Float = 300
    var u: Float = 0, v: Float = 0
    var jit: Float = 0
    var yOff: Float = 0
    var age = 0
    var mom = false
    var prev = SIMD3<Float>(0, 0, 0)
}

struct FastRNG {
    var state: UInt64 = 0x9E3779B97F4A7C15
    mutating func next() -> UInt64 {
        state ^= state << 13
        state ^= state >> 7
        state ^= state << 17
        return state
    }
    mutating func unit() -> Float { Float(next() >> 40) / Float(1 << 24) }
}


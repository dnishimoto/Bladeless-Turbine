//
//  DataStructures.swift
//  Bladeless Turbine
//
//  Created by David Nishimoto on 11/9/26.
//

import Foundation
import SwiftUI
import SceneKit

struct EngineConfiguration {
    var diskCount = 9
    var diskRadius: Float = 1.0
    var hubRadius: Float = 0.12
    var diskSpacing: Float = 0.09
    var diskThickness: Float = 0.012
    var holeCount = 3
    var holeInnerRadius: Float = 0.24
    var holeOuterRadius: Float = 0.52
    var holeHalfAngle: Float = 24.0 * .pi / 180.0

    var gamma: Float = 1.4
    var ambientTemp: Float = 288.15
    var ambientPressure: Float = 101.3
    var compressorRatio: Float = 4.0
    var fuelFlowRate: Float = 0.05
    var combustionEfficiency: Float = 0.98

    var viscousCoupling: Float = 0.22
    var radialFlowRate: Float = 0.14
    var torqueGain: Float = 0.00045
    var diskFriction: Float = 0.28

    var gapCount: Int { diskCount - 1 }
    var sectorCount = 48
    var sectorAngle: Float { 2.0 * .pi / Float(sectorCount) }

    var maxParticles = 1200
    var tickRate: Float = 30.0
}

// MARK: - Gas Stages

enum EngineStage: Int, CaseIterable, Identifiable {
    case intake, compression, combustion, turbine, exhaust

    var id: Int { rawValue }

    var title: String {
        switch self {
        case .intake: return "AIR INTAKE"
        case .compression: return "COMPRESSION"
        case .combustion: return "COMBUSTION CHAMBER"
        case .turbine: return "BLADELESS TURBINE"
        case .exhaust: return "EXHAUST NOZZLE"
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

// MARK: - Jet Engine Particle Model (Renamed to avoid conflict with TurbineCA)

struct JetGasParticle {
    var active = false
    var stage: EngineStage = .intake

    var radius: Float = 0.9
    var angle: Float = 0
    var axialPosition: Float = 3.5

    var temperature: Float = 288.15
    var pressure: Float = 101.3
    var velocity: Float = 50.0
    var angularVelocity: Float = 0

    var node: SCNNode?
    var age: Float = 0
}


// MARK: - Configuration

struct TurbineConfig {
    // Disc stack
    var diskCount = 9
    var diskRadius: Float = 1.0
    var hubRadius: Float = 0.12
    var pitch: Float = 0.09
    var diskThickness: Float = 0.012

    // Tesla turbine CA grid
    var ringCount = 20
    var sectorCount = 48            // keep a multiple of holeCount
    var innerRingRadius: Float = 0.24
    var holeCount = 3               // three openings per disc
    var holeInnerRadius: Float = 0.24
    var holeOuterRadius: Float = 0.52
    var holeHalfAngle: Float = 24 * .pi / 180
    var cellCapacity = 3
    var nozzleSectors = 3           // rim sectors fed by the tangential inlet
    var turbineAmbientPerCell: Float = 0.15

    // Gas / engine network
    var ambientParticles: Float = 6          // particles per unit volume at ambient
    var ambientPressure: Float = 101.325     // kPa
    var ambientTemp: Float = 300
    var tickRate: Float = 20
    var maxParticles = 2400
    var airRefillPerTick = 60

    // Compressor
    var designOmega: Float = 3.5
    var maxOmega: Float = 5
    var compressorHeadPerStage: Float = 0.7  // × ambient pressure at design speed
    var compressionHeat: Float = 0.03
    var compressorLoadGain: Float = 0.05

    // Combustor
    var fuelEnergyPerTick: Float = 11000     // K·particles per tick at fuel = 1
    var fuelGasPerTick: Float = 5            // fuel mass becoming gas per tick at fuel = 1
    var maxFlameTemp: Float = 2000
    var maxFuelFlowGramsPerSec: Float = 60

    // Shaft
    var turbineTorqueGain: Float = 0.010
    var shaftFriction: Float = 0.35
    var starterTorque: Float = 2.5
    var starterCutoff: Float = 0.55          // fraction of design speed
    var displayDesignRPM: Float = 24000      // what 100 % design speed is labelled as
    var thrustPerUnit: Float = 0.02

    // Duct geometry (shared with the scene builder)
    var chainACells = 16
    var chainBCells = 10
    var jetCellLength: Float = 0.12

    var gapCount: Int { diskCount - 1 }
    var sectorAngle: Float { 2 * .pi / Float(sectorCount) }
    var ringWidth: Float { (diskRadius - innerRingRadius) / Float(ringCount) }
}
// MARK: - Particle

// MARK: - Particle

struct GasParticle {
    var stage: GasStage = .free
    var j = 0                       // duct cell (intake / collector)
    var g = 0, r = 0, t = 0         // turbine cell
    var temp: Float = 300           // kelvin
    var u: Float = 0, v: Float = 0  // position across the duct, -1...1
    var jit: Float = 0              // sub-cell offset along the duct
    var yOff: Float = 0             // vertical jitter inside a disc gap, in pitches
    var age = 0
    var mom = false                 // moved last tick (momentum)
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
    /// Uniform in [0, 1)
    mutating func unit() -> Float { Float(next() >> 40) / Float(1 << 24) }
}


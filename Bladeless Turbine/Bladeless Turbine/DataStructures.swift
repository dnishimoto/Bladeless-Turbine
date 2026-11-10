//
//  DataStructures.swift
//  Bladeless Turbine
//
//  Shared simulation data structures
//

import Foundation
import SwiftUI
import SceneKit


struct TurbineConfig {
    
    var inletPressure: Float = 145_000         // High ram-air total pressure recovery at the face
    var outletPressure: Float = 101_325
    var ambientPressure: Float = 101.325
    var axialPressureDrop: Float = 350.0       // High-speed pressure drop across core stages
    
    // High-speed flow parameters matching fighter intake velocity profiles
    var maxRadialSpeed: Float = 220.0          // High axial/radial ingestion speed (~Mach 0.6–0.8 face velocity)
    var maxTangentialSpeed: Float = 180.0
    var inletTangentialSpeed: Float = 80.0     // Intensive fan/compressor face swirl component
    
    // Scaled mass flow reference matching ~112 kg/s class engine core
    var referenceMassFlow: Float = 0.12

    // MARK: Rotor Geometry

    var diskCount: Int = 9
    var diskRadius: Float = 1.0
    var hubRadius: Float = 0.12
    var pitch: Float = 0.09
    var diskThickness: Float = 0.012

    // MARK: Cellular Automaton Mesh

    var ringCount: Int = 20
    var sectorCount: Int = 48
    var innerRingRadius: Float = 0.24

    // MARK: Exhaust Openings

    var holeCount: Int = 3
    var holeInnerRadius: Float = 0.24
    var holeOuterRadius: Float = 0.52
    var holeHalfAngle: Float = 24 * Float.pi / 180

    // MARK: Gas Particle Rendering
    var cellCapacity: UInt8 = 6              // Increased cell packing capacity for denser flow
        var maxParticles: Int = 4000             // Increased from 1400 to 4000 gas particles
        var maxInjectPerTick: Float = 15.0       // Increased injection rate per tick
        var nozzleSectors: Int = 2
        var tickRate: Float = 20
        var exhaustLife: Int = 24
    
    // MARK: Air Properties

    var gasConstant: Float = 287.05
    var gamma: Float = 1.4

   
    var inletTemperature: Float = 950
    var ambientTemperature: Float = 288.15

    // MARK: Fuel & Combustion

    var fuelGasPerTick: Float = 5.0
  
    // MARK: Flow Physics

    var effectiveFlowGap: Float = 0.0008
    var thermalDiffusivity: Float = 2.2e-5
   
    // MARK: Rotor Mechanics

    var rotorInertia: Float = 0.08
    var rotorDrag: Float = 0.08
    var diskFriction: Float = 0.08
 
 
    // MARK: Derived Mesh Values

    var gapCount: Int {
        max(1, diskCount - 1)
    }

    var sectorAngle: Float {
        2 * Float.pi / Float(max(1, sectorCount))
    }

    var ringWidth: Float {
        (diskRadius - innerRingRadius) /
        Float(max(1, ringCount))
    }

 
}


// MARK: - Gas Particle

struct GasParticle {

    // Particle state
    var active: Bool = false
    var stage: GasStage = .free

    // Current CA cell
    var g: Int = 0
    var r: Int = 0
    var t: Int = 0

    // Previous CA cell for interpolation
    var pg: Int = 0
    var pr: Int = 0
    var pt: Int = 0

    // Thermodynamic state
    var temp: Float = 288.15

    // Visual vertical offset
    var yOffset: Float = 0

    // Exhaust tracking
    var exhaustAge: Int = -1

    // Sub-cell movement remainders
    var radialRemainder: Float = 0
    var angularRemainder: Float = 0
    var axialRemainder: Float = 0
}


enum GasStage: Int, CaseIterable, Codable {
    case free
    case intake
    case turbine
    case collector
    case plume
}

struct FastRNG {

    var state: UInt64 = 0x9E3779B97F4A7C15

    mutating func next() -> UInt64 {
        state ^= state << 13
        state ^= state >> 7
        state ^= state << 17
        return state
    }

    mutating func unit() -> Float {
        Float(next() >> 40) / Float(1 << 24)
    }
}

struct FlowCell {
    var density: Float = 1.2
    var pressure: Float = 101_325
    var temperature: Float = 288.15

    var radialVelocity: Float = 0
    var tangentialVelocity: Float = 0
    var axialVelocity: Float = 0

    var totalSpecificEnergy: Float = 0
}

// MARK: - Pressure Measurement Station

struct PressureStation: Identifiable {

    let id: Int
    let name: String

    var kPa: Float
    var tempK: Float
}

// MARK: - Engine Stages

enum EngineStage: Int, CaseIterable, Identifiable {

    case intake
    case compressor
    case combustor
    case turbine
    case collector
    case exhaust

    var id: Int {
        rawValue
    }

    var title: String {
        switch self {
        case .intake: return "Intake"
        case .compressor: return "Compressor"
        case .combustor: return "Combustor"
        case .turbine: return "Turbine"
        case .collector: return "Collector"
        case .exhaust: return "Exhaust"
        }
    }

    var color: UIColor {
        switch self {
        case .intake: return .systemBlue
        case .compressor: return .systemCyan
        case .combustor: return .systemYellow
        case .turbine: return .systemOrange
        case .collector: return .systemRed
        case .exhaust: return .systemPink
        }
    }
}



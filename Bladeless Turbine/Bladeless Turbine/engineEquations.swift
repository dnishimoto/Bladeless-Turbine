//
//  EngineEquations.swift
//  Bladeless Turbine
//
//  Closed-form gas-turbine cycle (SI units). Air intake, shaft torque and exhaust
//  force all come from these equations, not from tuned gains:
//
//  AMBIENT (ISA)  T₀ = 288.15 − 6.5e-3·h        p₀ = 101325 (T₀/288.15)^5.2559
//                 a₀ = √(γRT₀)   M₀ = V/a₀
//
//  INLET RAM      T₀₁ = T₀(1 + 0.2 M₀²)         p₀₁ = η_r · p₀ (1 + 0.2 M₀²)^3.5
//
//  COMPRESSOR     N_c = N/N_d · √(288.15/T₀₁)    (corrected speed)
//                 π_c = 1 + (PR − 1)·N_c²
//                 T₀₂ = T₀₁ [1 + (π_c^((γ−1)/γ) − 1)/η_c]    p₀₂ = π_c·p₀₁
//
//  COMBUSTOR      f = fuel · f_max               (fuel–air ratio)
//                 T₀₃ = (T₀₂ + η_b·f·LHV/cp)/(1 + f)     p₀₃ = p₀₂(1 − Δp_b)
//
//  MASS FLOW      The Tesla-turbine inlet manifold is a choked throat of area A_th:
//                 ṁ_e = A_th · p₀₃/√T₀₃ · √(γ/R) · (2/(γ+1))^((γ+1)/(2(γ−1)))
//                 ṁ_air = ṁ_e/(1+f)       ṁ_fuel = f·ṁ_air
//                 fan-face Mach (derived) M₁ = ṁ_air/(ρ₀₁·A₁·a₀₁)
//
//  TESLA TURBINE  T₅ = T₀₃ [1 − η_t (1 − π_t^(−(γ−1)/γ))]   p₅ = p₀₃/π_t
//                 P_t = ṁ_e·cp·(T₀₃ − T₅)       P_c = ṁ_air·cp·(T₀₂ − T₀₁)       τ = P/Ω
//                 π_t solved so the exit nozzle passes exactly ṁ_e.
//
//  SHAFT          J·dΩ/dt = τ_t − τ_c − τ_friction + τ_starter
//
//  NOZZLE / FORCE choked when p₅/p₀ ≥ ((γ+1)/2)^(γ/(γ−1)); V_e, p_e as before
//                 F_net = ṁ_e·V_e − ṁ_air·V₀ + (p_e − p₀)·A_e
//

import Foundation

struct CycleResult {
    var speedRatio: Float = 0

    // Ambient / flight
    var ambientT: Float = 288.15
    var ambientP: Float = 101_325
    var flightMach: Float = 0
    var ramDrag: Float = 0             // N, ṁ_air·V₀

    // Intake
    var intakeAirFlow: Float = 0       // kg/s
    var intakeMach: Float = 0
    var fuelAirRatio: Float = 0
    var fuelFlow: Float = 0            // kg/s
    var exhaustFlow: Float = 0         // kg/s

    // Stations (static / total as noted)
    var t1: Float = 288.15, p1: Float = 101_325
    var compressorPR: Float = 1
    var t2: Float = 288.15, p2: Float = 101_325
    var t3: Float = 288.15, p3: Float = 101_325      // combustor exit = Tesla rim
    var turbinePR: Float = 1
    var t5: Float = 288.15, p5: Float = 101_325      // Tesla turbine exit / nozzle entry
    var te: Float = 288.15, pe: Float = 101_325      // nozzle exit

    // Shaft
    var compressorPower: Float = 0     // W
    var turbinePower: Float = 0        // W
    var compressorTorque: Float = 0    // N·m
    var turbineTorque: Float = 0       // N·m
    var frictionTorque: Float = 0      // N·m

    // Exhaust force
    var exhaustVelocity: Float = 0     // m/s
    var nozzleExitMach: Float = 0
    var momentumThrust: Float = 0      // N
    var pressureThrust: Float = 0      // N
    var thrust: Float = 0              // N
    var choked = false
}

struct EngineEquations {
    let cfg: TurbineConfig

    private let g: Double, r: Double, cp: Double, k: Double
    private let intakeArea: Double
    private let frictionAtDesign: Double

    init(config: TurbineConfig) {
        cfg = config
        g = Double(config.gamma)
        r = Double(config.gasConstant)
        cp = g * r / (g - 1)
        k = (g - 1) / g
        let tip = Double(config.intakeTipRadius), hub = Double(config.intakeHubRadius)
        intakeArea = Double.pi * (tip * tip - hub * hub)

        // Friction torque at design speed = a fraction of the design compressor torque.
        let t1 = 288.15
        let pr = Double(max(1, config.designOverallPR))
        let t2 = t1 * (1 + (pow(pr, (g - 1) / g) - 1) / Double(config.compressorEfficiency))
        frictionAtDesign = Double(config.frictionFraction) * Double(config.referenceMassFlow)
            * cp * (t2 - t1) / Double(config.physicalDesignOmega)
    }

    /// International Standard Atmosphere (troposphere + isothermal lower stratosphere).
    static func atmosphere(altitude h: Double) -> (t: Double, p: Double) {
        let t = 288.15 - 0.0065 * min(max(0, h), 11_000)
        return (t, 101_325 * pow(t / 288.15, 5.2559) * (h > 11_000 ? exp(-(h - 11_000) / 6_340) : 1))
    }

    private func chokedFlowFunction() -> Double {
        (g / r).squareRoot() * pow(2 / (g + 1), (g + 1) / (2 * (g - 1)))
    }

    // Mass flow capacity (kg/s) of the exit nozzle at total p05, T05, exhausting to p0
    private func nozzleFlow(p05: Double, t05: Double, p0: Double) -> Double {
        let a = Double(cfg.nozzleExitArea)
        let critical = pow((g + 1) / 2, g / (g - 1))
        if p05 / p0 >= critical { return a * p05 / t05.squareRoot() * chokedFlowFunction() }
        let x = p0 / p05
        return a * p05 / t05.squareRoot() * (2 * g / (r * (g - 1))).squareRoot()
            * pow(x, 1 / g) * max(0, 1 - pow(x, k)).squareRoot()
    }

    // MARK: Solve the cycle at a shaft speed, fuel command and flight condition

    // MARK: Solve the cycle at a shaft speed, fuel command and flight condition

        func solve(speedRatio: Float, fuelCommand: Float, overallPR: Float,
                   altitude: Float = 0, airspeed: Float = 0) -> CycleResult {
            var out = CycleResult()
            let s = Double(max(0.02, speedRatio))
            let pr = Double(max(1, overallPR))
            out.speedRatio = Float(s)

            // Ambient and ram recovery
            let (t0, p0) = Self.atmosphere(altitude: Double(altitude))
            let a0 = (g * r * t0).squareRoot()
            let v0 = Double(max(0, airspeed))
            let m0 = v0 / a0
            let ram = 1 + 0.5 * (g - 1) * m0 * m0
            let t1 = t0 * ram
            let p1 = p0 * pow(ram, g / (g - 1)) * Double(cfg.inletRecovery)

            // Compressor (corrected speed)
            let sc = s * (288.15 / t1).squareRoot()
            let pic = 1 + (pr - 1) * sc * sc
            let t2 = t1 * (1 + (pow(pic, k) - 1) / Double(cfg.compressorEfficiency))
            let p2 = pic * p1
            let omega = s * Double(cfg.physicalDesignOmega)

            // Combustor energy balance
            let f = Double(max(0, min(1, fuelCommand))) * Double(cfg.maxFuelAirRatio)
            let p3 = p2 * (1 - Double(cfg.burnerPressureLoss))
            let t3 = (t2 + Double(cfg.burnerEfficiency) * f * Double(cfg.lowerHeatingValue) / cp) / (1 + f)

            // Mass flow: choked Tesla inlet throat
            let me = Double(cfg.hotThroatArea) * p3 / t3.squareRoot() * chokedFlowFunction()
            let ma = me / (1 + f)
            let wc = ma * cp * (t2 - t1)

            // Turbine expansion ratio from exit-nozzle continuity (bisection)
            var lo = 1.0001, hi = max(1.0002, p3 / p0)
            let etaT = Double(cfg.turbineEfficiency)
            for _ in 0..<40 {
                let mid = 0.5 * (lo + hi)
                let t05 = t3 * (1 - etaT * (1 - pow(mid, -k)))
                if nozzleFlow(p05: p3 / mid, t05: t05, p0: p0) > me { lo = mid } else { hi = mid }
            }
            let pit = 0.5 * (lo + hi)
            let t5 = t3 * (1 - etaT * (1 - pow(pit, -k)))
            let p5 = p3 / pit
            let wt = me * cp * (t3 - t5)

            // Nozzle exit and net force (updated with mass flow continuity)
            let critical = pow((g + 1) / 2, g / (g - 1))
            let choked = p5 / p0 >= critical
            var te: Double, pe: Double, ve: Double, mExit: Double
            let aE = Double(cfg.nozzleExitArea)

            if choked {
                te = 2 * t5 / (g + 1)
                pe = p5 / critical
                ve = (g * r * te).squareRoot()
                mExit = 1
            } else {
                pe = p0
                let idealVe = (2 * cp * t5 * max(0, 1 - pow(p0 / max(p0, p5), k))).squareRoot()
                let tentativeTe = t5 - idealVe * idealVe / (2 * cp)
                let rhoE = pe / (r * max(180, tentativeTe))
                
                ve = me / max(1.0e-5, rhoE * aE)
                te = t5 - ve * ve / (2 * cp)
                mExit = ve / (g * r * max(1, te)).squareRoot()
            }
            let momentum = me * ve - ma * v0
            let pressure = (pe - p0) * aE

            // Derived fan-face Mach from continuity
            let rho01 = p1 / (r * t1), a01 = (g * r * t1).squareRoot()

            out.ambientT = Float(t0); out.ambientP = Float(p0)
            out.flightMach = Float(m0)
            out.ramDrag = Float(ma * v0)
            out.intakeAirFlow = Float(ma)
            out.intakeMach = Float(ma / (rho01 * intakeArea * a01))
            out.fuelAirRatio = Float(f)
            out.fuelFlow = Float(ma * f)
            out.exhaustFlow = Float(me)
            out.t1 = Float(t1); out.p1 = Float(p1)
            out.compressorPR = Float(pic)
            out.t2 = Float(t2); out.p2 = Float(p2)
            out.t3 = Float(t3); out.p3 = Float(p3)
            out.turbinePR = Float(pit)
            out.t5 = Float(t5); out.p5 = Float(p5)
            out.te = Float(te); out.pe = Float(pe)
            out.compressorPower = Float(wc)
            out.turbinePower = Float(wt)
            out.compressorTorque = Float(wc / omega)
            out.turbineTorque = Float(wt / omega)
            out.frictionTorque = Float(frictionAtDesign * s * s)
            out.exhaustVelocity = Float(ve)
            out.nozzleExitMach = Float(mExit)
            out.momentumThrust = Float(momentum)
            out.pressureThrust = Float(pressure)
            out.thrust = Float(momentum + pressure)
            out.choked = choked
            return out
        }
}

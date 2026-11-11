//
//  EngineLayout.swift
//  Bladeless Turbine
//
//  Shape of the engine, shared by the SceneKit builders (where each part sits)
//  and the gas tracers (where each particle is drawn), so the gas follows the casing.
//
//  Engine-local frame: the engine axis is +X and the gas flows toward +X.
//  The scene rotates this frame so the flow runs left → right on screen.
//
//      INTAKE        COMPRESSION            COMBUSTION   TESLA TURBINE    EXHAUST
//     fan + spinner  tapering drum,         annular      discs on the     tail cone +
//                    4 rotor/stator rows    combustor    same shaft       converging nozzle
//     |------------ cold section ----------|-------------- hot section -----------------|
//
//  The simulated disc radius (cfg.diskRadius = 1.0) is drawn at `discScale` so the
//  stack fits inside the engine casing; angles and ring order are unchanged.
//

import Foundation
import simd

struct EngineLayout {
    let cfg: TurbineConfig

    // MARK: Drawn disc stack

    let discScale: Float = 0.62
    var rimRadius: Float { cfg.diskRadius * discScale }
    var hubRadius: Float { cfg.hubRadius * discScale }
    var holeInner: Float { cfg.holeInnerRadius * discScale }
    var holeOuter: Float { cfg.holeOuterRadius * discScale }
    var pitch: Float { cfg.pitch * discScale }
    var diskThickness: Float { cfg.diskThickness * discScale }

    // MARK: Axial stations (local X)

    let xAirStart: Float = -3.7
    let xNose: Float = -3.45
    let xLip: Float = -3.2
    let xCompStart: Float = -2.45
    let xCompEnd: Float = -0.75
    let xCombEnd: Float = 0.05
    let stackCenterX: Float = 0.62
    let xExhaustEnd: Float = 2.7

    var stackHalfSpan: Float { Float(cfg.gapCount) * pitch / 2 }
    var xStackStart: Float { stackCenterX - stackHalfSpan }
    var xStackEnd: Float { stackCenterX + stackHalfSpan }
    var turbineCasingRadius: Float { rimRadius + 0.12 }

    func lerp(_ a: Float, _ b: Float, _ t: Float) -> Float { a + (b - a) * max(0, min(1, t)) }

    func diskX(_ i: Int) -> Float { stackCenterX + (Float(i) - Float(cfg.diskCount - 1) / 2) * pitch }
    func gapX(_ g: Float) -> Float { stackCenterX + (g + 0.5 - Float(cfg.gapCount) / 2) * pitch }

    // MARK: Front duct, continuous coordinate c in 0...16
    //   0-2 intake · 2-6 compressor · 6-12 combustor · 12-16 manifold into the disc rim

    func frontX(_ c: Float) -> Float {
        if c <= 1 { return lerp(xAirStart, xLip, c) }
        if c <= 2 { return lerp(xLip, xCompStart, c - 1) }
        if c <= 6 { return lerp(xCompStart, xCompEnd, (c - 2) / 4) }
        if c <= 12 { return lerp(xCompEnd, xCombEnd, (c - 6) / 6) }
        return lerp(xCombEnd, xStackStart - 0.04, (c - 12) / 4)
    }

    func frontInner(_ c: Float) -> Float {
        if c <= 1 { return lerp(0.0, 0.10, c) }
        if c <= 2 { return 0.10 }
        if c <= 6 { return lerp(0.10, 0.26, (c - 2) / 4) }
        return 0.26
    }

    func frontOuter(_ c: Float) -> Float {
        if c <= 1 { return lerp(0.66, 0.56, c) }
        if c <= 2 { return lerp(0.56, 0.52, c - 1) }
        if c <= 6 { return lerp(0.52, 0.40, (c - 2) / 4) }
        if c <= 7 { return lerp(0.40, 0.64, c - 6) }
        if c <= 12 { return lerp(0.64, 0.66, (c - 7) / 5) }
        return lerp(0.66, rimRadius + 0.05, (c - 12) / 4)
    }

    // MARK: Rear duct, continuous coordinate c in 0...10 (collector, then converging nozzle)

    func rearX(_ c: Float) -> Float { lerp(xStackEnd + 0.06, xExhaustEnd, c / 10) }
    func rearInner(_ c: Float) -> Float { lerp(0.20, 0.0, c / 10) }
    func rearOuter(_ c: Float) -> Float {
        c <= 3 ? lerp(holeOuter, 0.50, c / 3) : lerp(0.50, 0.36, (c - 3) / 7)
    }

    // MARK: Tracer placement

    /// Local-frame position of a gas tracer.
    ///  - ring: continuous ring coordinate, 0 (centre) … 20 (rim); upstream engine stations are the outer rings
    ///  - gap:  continuous disc-gap coordinate
    ///  - angle: swirl angle around the axis (matches a disc node turned so its normal is +X)
    ///  - frac: fixed 0…1 position of this particle across the duct
    ///  - exhaustAge: nil while in the engine, else ticks since it left the discs
    func tracerPosition(ring: Float, gap: Float, angle: Float, frac: Float, exhaustAge: Float?) -> SIMD3<Float> {
        var a: Float, r: Float

        if let age = exhaustAge {
            let life = Float(cfg.exhaustLife)
            let ductAge = life * 0.55
            if age <= ductAge {
                let c = age / ductAge * 10
                a = rearX(c)
                r = lerp(rearInner(c) + 0.01, rearOuter(c) - 0.01, frac)
            } else {
                let past = age - ductAge
                a = xExhaustEnd + past * 0.14
                r = lerp(0, 0.36, frac) * (1 + 0.12 * past)
            }
        } else if ring >= 7 {
            // Front duct: rings 20 → 7 stand for intake → compressor → combustor → manifold.
            let c = max(0, min(15.99, (20 - ring) / 13 * 16))
            if c < 12 {
                a = frontX(c)
                r = lerp(frontInner(c) + 0.02, frontOuter(c) - 0.02, frac)
            } else {
                let s = (c - 12) / 4
                let r12 = lerp(frontInner(12) + 0.02, frontOuter(12) - 0.02, frac)
                a = lerp(frontX(12), gapX(gap), s)
                r = lerp(r12, rimRadius + 0.03, s)
            }
        } else {
            // Inside the Tesla stack: spiral from the rim toward the openings, between the discs.
            let u = max(0, min(1, (7 - ring) / 6))
            a = gapX(gap) + (frac - 0.5) * pitch * 0.7
            r = lerp(rimRadius, (holeInner + holeOuter) / 2, u)
        }
        return SIMD3(a, r * sin(angle), -r * cos(angle))
    }

    // MARK: Profiles for the lathe-built shells (x, radius)

    var coldCasingProfile: [SIMD2<Float>] {
        [SIMD2(xLip - 0.12, 0.60), SIMD2(xLip, 0.56), SIMD2(xCompStart, 0.52),
         SIMD2(xCompEnd, 0.40), SIMD2(xCompEnd + 0.12, 0.62)]
    }

    var hotCasingProfile: [SIMD2<Float>] {
        [SIMD2(xCompEnd + 0.12, 0.62), SIMD2(xCombEnd - 0.1, 0.66), SIMD2(xCombEnd + 0.12, turbineCasingRadius),
         SIMD2(xStackEnd + 0.04, turbineCasingRadius),
         SIMD2(xStackEnd + 0.20, 0.52), SIMD2(1.4, 0.50), SIMD2(xExhaustEnd, 0.36)]
    }

    var drumProfile: [SIMD2<Float>] {
        [SIMD2(xNose, 0.0), SIMD2(xNose + 0.12, 0.07), SIMD2(xLip + 0.1, 0.11),
         SIMD2(xCompStart, 0.11), SIMD2(xCompEnd, 0.26), SIMD2(xCombEnd - 0.05, 0.26),
         SIMD2(xCombEnd, hubRadius)]
    }

    var tailConeProfile: [SIMD2<Float>] {
        [SIMD2(xStackEnd + 0.06, 0.20), SIMD2(1.3, 0.17), SIMD2(2.3, 0.05), SIMD2(2.56, 0.0)]
    }
}

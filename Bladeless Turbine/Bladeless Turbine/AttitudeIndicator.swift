//
//  AttitudeIndicator.swift
//  Bladeless Turbine
//
//  Created by David Nishimoto on 11/12/26.
//
import SwiftUI
import Foundation


/// Artificial horizon: sky/ground tilt with bank and slide with pitch; the fixed wings show the jet.
/// Turns green and reads LEVEL when bank and pitch are both within 2°.
struct AttitudeIndicator: View {
    let bank: Float      // degrees, + = right wing down
    let pitch: Float     // degrees, + = nose up

    private let size: CGFloat = 78
    private let pixelsPerDegree: CGFloat = 1.6

    private var isLevel: Bool { abs(bank) < 2 && abs(pitch) < 2 }

    var body: some View {
        VStack(spacing: 2) {
            ZStack {
                // Horizon: pitch slides it, bank rotates it the opposite way to the jet.
                ZStack {
                    VStack(spacing: 0) {
                        Rectangle().fill(Color(red: 0.25, green: 0.55, blue: 0.95))
                        Rectangle().fill(Color(red: 0.55, green: 0.38, blue: 0.20))
                    }
                    .frame(width: size * 3, height: size * 6)
                    Rectangle().fill(.white).frame(width: size * 3, height: 1.5)

                    // Pitch ladder every 10°
                    ForEach([-20, -10, 10, 20], id: \.self) { deg in
                        Rectangle().fill(.white.opacity(0.8))
                            .frame(width: abs(deg) == 20 ? 30 : 18, height: 1)
                            .offset(y: -CGFloat(deg) * pixelsPerDegree)
                    }
                }
                .offset(y: CGFloat(pitch) * pixelsPerDegree )
                .rotationEffect(.degrees(Double(bank)))

                // Fixed aircraft symbol
                HStack(spacing: 0) {
                    Rectangle().fill(.yellow).frame(width: 20, height: 3)
                    Circle().fill(.yellow).frame(width: 6, height: 6).padding(.horizontal, 4)
                    Rectangle().fill(.yellow).frame(width: 20, height: 3)
                }

                // Top roll pointer
                Image(systemName: "arrowtriangle.down.fill")
                    .font(.system(size: 7))
                    .foregroundStyle(.yellow)
                    .offset(y: -size / 2 + 5)
            }
            .frame(width: size, height: size)
            .clipShape(Circle())
            .overlay(Circle().stroke(isLevel ? Color.green : Color.white.opacity(0.6), lineWidth: 2))

            Text(isLevel ? "LEVEL" : String(format: "%+.0f° %+.0f°", bank, pitch))
                .font(.system(size: 8, weight: .bold, design: .monospaced))
                .foregroundStyle(isLevel ? .green : .secondary)
        }
    }
}

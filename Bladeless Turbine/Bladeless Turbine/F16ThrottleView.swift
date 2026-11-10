//
//  F16ThrottleView.swift
//  Bladeless Turbine
//

import SwiftUI

struct F16ThrottleView: View {
    @Binding var throttle: Double // 0.0 to 1.0
    private let trackHeight: CGFloat = 140.0

    var body: some View {
        VStack(spacing: 6) {
            Text("THROTTLE")
                .font(.system(size: 10, weight: .bold, design: .monospaced))
                .foregroundStyle(.yellow)

            HStack(spacing: 14) {
                // Detent Scale Labels
                VStack(alignment: .trailing, spacing: 22) {
                    Text("A/B").font(.system(size: 8, weight: .bold, design: .monospaced)).foregroundStyle(.orange)
                    Text("MIL").font(.system(size: 8, weight: .bold, design: .monospaced)).foregroundStyle(.white)
                    Text("IDLE").font(.system(size: 8, weight: .bold, design: .monospaced)).foregroundStyle(.cyan)
                    Text("OFF").font(.system(size: 8, weight: .bold, design: .monospaced)).foregroundStyle(.secondary)
                }

                // Vertical Throttle Slider Track & Handle
                ZStack(alignment: .bottom) {
                    // Track Background
                    RoundedRectangle(cornerRadius: 6)
                        .fill(Color.black.opacity(0.7))
                        .frame(width: 18, height: trackHeight)
                        .overlay(
                            RoundedRectangle(cornerRadius: 6)
                                .stroke(Color.white.opacity(0.3), lineWidth: 1)
                        )

                    // Active Throttle Fill (grows upward)
                    RoundedRectangle(cornerRadius: 6)
                        .fill(
                            LinearGradient(
                                colors: [Color.cyan, Color.orange, Color.red],
                                startPoint: .bottom,
                                endPoint: .top
                            )
                        )
                        .frame(width: 18, height: max(6, CGFloat(throttle) * trackHeight))

                    // Detent Tick Marks
                    VStack(spacing: 0) {
                        Spacer()
                        ForEach([0.0, 0.35, 0.7, 1.0], id: \.self) { pos in
                            Rectangle()
                                .fill(Color.white.opacity(0.6))
                                .frame(width: 22, height: 2)
                            if pos < 1.0 { Spacer() }
                        }
                    }
                    .frame(height: trackHeight)

                    // Physical Throttle Grip Knob overlay
                    Capsule()
                        .fill(LinearGradient(colors: [Color.gray, Color.secondary], startPoint: .top, endPoint: .bottom))
                        .frame(width: 32, height: 20)
                        .overlay(Capsule().stroke(Color.white.opacity(0.8), lineWidth: 1.5))
                        .shadow(color: .black.opacity(0.5), radius: 3, x: 0, y: 2)
                        .offset(y: -CGFloat(throttle) * (trackHeight - 20))
                }
                .frame(width: 36, height: trackHeight)
                .contentShape(Rectangle())
                .gesture(
                    DragGesture(minimumDistance: 0)
                        .onChanged { value in
                            let dragFraction = 1.0 - (value.location.y / trackHeight)
                            throttle = min(1.0, max(0.0, Double(dragFraction)))
                        }
                )

                // Throttle Percentage & Detent Status Display
                VStack(alignment: .leading, spacing: 3) {
                    Text("\(Int(throttle * 100))%")
                        .font(.system(size: 16, weight: .bold, design: .monospaced))
                        .foregroundStyle(throttle > 0.7 ? .orange : .white)
                    Text(throttle > 0.85 ? "MAX A/B" : throttle > 0.7 ? "MIL PWR" : throttle > 0.1 ? "FLIGHT" : "CUTOFF")
                        .font(.system(size: 8.5, weight: .bold, design: .monospaced))
                        .foregroundStyle(throttle > 0.85 ? .red : .secondary)
                }
                .frame(width: 65, alignment: .leading)
            }
        }
        .padding(10)
        .background(.ultraThinMaterial, in: RoundedRectangle(cornerRadius: 12))
    }
}

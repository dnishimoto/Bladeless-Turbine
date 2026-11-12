
//
//  F16ThrottleView.swift
//  Bladeless Turbine
//

import SwiftUI

struct F16ThrottleView: View {

    @Binding var throttle: Double

    private let trackHeight: CGFloat = 140

    private var normalizedThrottle: Double {
        min(1.0, max(0.0, throttle))
    }

    private var throttleColor: Color {
        normalizedThrottle > 0.85 ? .red :
        normalizedThrottle > 0.70 ? .orange : .cyan
    }

    private var throttleStatus: String {
        if normalizedThrottle > 0.85 {
            return "MAX A/B"
        } else if normalizedThrottle > 0.70 {
            return "MIL PWR"
        } else if normalizedThrottle > 0.10 {
            return "FLIGHT"
        } else if normalizedThrottle > 0.0 {
            return "IDLE"
        } else {
            return "CUTOFF"
        }
    }

    var body: some View {

        VStack(spacing: 6) {

            Text("THROTTLE")
                .font(.system(size: 10, weight: .bold, design: .monospaced))
                .foregroundStyle(.yellow)

            HStack(spacing: 14) {

                // Detent labels
                VStack(alignment: .trailing, spacing: 22) {

                    Text("A/B")
                        .foregroundStyle(.orange)

                    Text("MIL")
                        .foregroundStyle(.white)

                    Text("IDLE")
                        .foregroundStyle(.cyan)

                    Text("OFF")
                        .foregroundStyle(.secondary)
                }
                .font(.system(size: 8, weight: .bold, design: .monospaced))

                // Vertical throttle track
                ZStack(alignment: .bottom) {

                    RoundedRectangle(cornerRadius: 6)
                        .fill(Color.black.opacity(0.7))
                        .frame(width: 18, height: trackHeight)
                        .overlay {
                            RoundedRectangle(cornerRadius: 6)
                                .stroke(
                                    Color.white.opacity(0.3),
                                    lineWidth: 1
                                )
                        }

                    // Throttle fill
                    VStack {
                        Spacer(minLength: 0)

                        RoundedRectangle(cornerRadius: 6)
                            .fill(
                                LinearGradient(
                                    colors: [.cyan, .orange, .red],
                                    startPoint: .bottom,
                                    endPoint: .top
                                )
                            )
                            .frame(
                                width: 18,
                                height: CGFloat(normalizedThrottle) * trackHeight
                            )
                    }
                    .frame(width: 18, height: trackHeight)

                    // Detent marks
                    ForEach(
                        [0.0, 0.35, 0.70, 1.0],
                        id: \.self
                    ) { position in

                        Rectangle()
                            .fill(Color.white.opacity(0.75))
                            .frame(width: 25, height: 2)
                            .offset(
                                y: -CGFloat(position) * trackHeight
                            )
                    }

                    // Physical throttle grip
                    Capsule()
                        .fill(
                            LinearGradient(
                                colors: [.gray, .secondary],
                                startPoint: .top,
                                endPoint: .bottom
                            )
                        )
                        .frame(width: 32, height: 20)
                        .overlay {
                            Capsule()
                                .stroke(
                                    Color.white.opacity(0.8),
                                    lineWidth: 1.5
                                )
                        }
                        .shadow(
                            color: .black.opacity(0.5),
                            radius: 3,
                            x: 0,
                            y: 2
                        )
                        .offset(
                            y: -CGFloat(normalizedThrottle) *
                                (trackHeight - 20)
                        )
                }
                .frame(width: 36, height: trackHeight)
                .contentShape(Rectangle())
                .gesture(
                    DragGesture(minimumDistance: 0)
                        .onChanged { value in

                            let fraction = 1.0 - (
                                value.location.y / trackHeight
                            )

                            throttle = min(
                                1.0,
                                max(0.0, Double(fraction))
                            )
                        }
                )
                .accessibilityLabel("Engine throttle")
                .accessibilityValue(
                    "\(Int(normalizedThrottle * 100)) percent"
                )

                // Throttle readout
                VStack(alignment: .leading, spacing: 3) {

                    Text("\(Int(normalizedThrottle * 100))%")
                        .font(
                            .system(
                                size: 16,
                                weight: .bold,
                                design: .monospaced
                            )
                        )
                        .foregroundStyle(throttleColor)

                    Text(throttleStatus)
                        .font(
                            .system(
                                size: 8.5,
                                weight: .bold,
                                design: .monospaced
                            )
                        )
                        .foregroundStyle(throttleColor)
                }
                .frame(width: 65, alignment: .leading)
            }
        }
        .padding(10)
        .background(
            .ultraThinMaterial,
            in: RoundedRectangle(cornerRadius: 12)
        )
    }
}

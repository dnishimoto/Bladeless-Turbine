//
//  BladelessTurbineView.swift
//  Bladeless Turbine
//

import SwiftUI
import SceneKit

// MARK: - SceneKit View

struct TurbineSceneView: UIViewRepresentable {
    let controller: TurbineSceneController

    func makeUIView(context: Context) -> SCNView {
        let view = SCNView()

        view.scene = controller.scene
        view.pointOfView = controller.cameraNode
        view.delegate = controller

        view.allowsCameraControl = true
        view.isPlaying = true
        view.preferredFramesPerSecond = 60
        view.antialiasingMode = .multisampling4X
        view.backgroundColor = .black

        return view
    }

    func updateUIView(_ uiView: SCNView, context: Context) {
    }
}

// MARK: - Pressure Overlay

struct PressureOverlay: View {
    let stations: [PressureStation]
    let ambient: Float

    private func colour(_ kPa: Float) -> Color {
        let ratio = kPa / max(0.001, ambient)

        if ratio < 0.97 {
            return Color(red: 0.4, green: 0.6, blue: 1.0)
        }

        if ratio < 1.5 {
            return Color(red: 0.4, green: 0.9, blue: 0.6)
        }

        if ratio < 3 {
            return Color(red: 1.0, green: 0.85, blue: 0.3)
        }

        if ratio < 5 {
            return Color(red: 1.0, green: 0.55, blue: 0.2)
        }

        return Color(red: 1.0, green: 0.3, blue: 0.25)
    }

    var body: some View {
        VStack(alignment: .leading, spacing: 4) {
            Text("STATION PRESSURES (kPa)")
                .font(.system(size: 9, weight: .bold, design: .monospaced))
                .foregroundStyle(.cyan)

            ForEach(stations) { station in
                HStack(spacing: 6) {
                    Text(station.name)
                        .frame(width: 115, alignment: .leading)

                    GeometryReader { geo in
                        let fraction = CGFloat(
                            min(
                                1,
                                max(
                                    0,
                                    (station.kPa / max(0.001, ambient)) / 7
                                )
                            )
                        )

                        ZStack(alignment: .leading) {
                            Capsule()
                                .fill(.white.opacity(0.12))

                            Capsule()
                                .fill(colour(station.kPa))
                                .frame(
                                    width: max(3, geo.size.width * fraction)
                                )
                        }
                    }
                    .frame(width: 50, height: 6)

                    Text(String(format: "%4.0f", station.kPa))
                        .frame(width: 30, alignment: .trailing)

                    Text(
                        String(
                            format: "%.1f×",
                            station.kPa / max(0.001, ambient)
                        )
                    )
                    .frame(width: 30, alignment: .trailing)
                    .foregroundStyle(.secondary)

                    Text(String(format: "%4.0fK", station.tempK))
                        .frame(width: 38, alignment: .trailing)
                        .foregroundStyle(.secondary)
                }
                .font(.system(size: 9, design: .monospaced))
            }
        }
        .padding(8)
        .background(
            .ultraThinMaterial,
            in: RoundedRectangle(cornerRadius: 8)
        )
    }
}

// MARK: - JoystickView

struct JoystickView: View {
    @Binding var roll: Double
    @Binding var pitch: Double
    @Binding var yaw: Double

    @State private var dragLocation: CGPoint = .zero
    @State private var isLongPressing: Bool = false
    @State private var initialYawValue: Double = 0.0
    @GestureState private var longPressState = false

    var body: some View {
        GeometryReader { geo in
            let size = min(geo.size.width, geo.size.height)
            let radius = size / 2

            ZStack {
                Circle()
                    .fill(Color.gray.opacity(0.2))
                    .frame(width: size, height: size)

                Circle()
                    .stroke(Color.white.opacity(0.4), lineWidth: 2)
                    .frame(width: size, height: size)

                Circle()
                    .fill(Color.cyan.opacity(0.7))
                    .frame(width: 40, height: 40)
                    .offset(x: dragLocation.x, y: dragLocation.y)
                    .animation(.easeOut(duration: 0.1), value: dragLocation)
            }
            .contentShape(Circle())
            .gesture(
                DragGesture(minimumDistance: 0)
                    .onChanged { value in
                        let loc = value.location
                        let center = CGPoint(x: radius, y: radius)
                        var delta = CGPoint(x: loc.x - center.x, y: loc.y - center.y)

                        // Clamp to circle radius
                        let distance = sqrt(delta.x * delta.x + delta.y * delta.y)
                        if distance > radius {
                            let scale = radius / distance
                            delta.x *= scale
                            delta.y *= scale
                        }

                        // Map delta.x/y from [-radius, radius] to [-1, 1]
                        let xNorm = Double(delta.x / radius)
                        let yNorm = Double(delta.y / radius)

                        // Update roll and pitch
                        roll = xNorm
                        pitch = -yNorm

                        // If long press active, use vertical drag for yaw
                        if isLongPressing {
                            // Calculate vertical drag delta from initial point
                            let verticalDrag = Double(value.translation.height / radius)
                            // Invert so dragging up increases yaw
                            yaw = (initialYawValue - verticalDrag).clamped(to: -1.0...1.0)
                        }

                        dragLocation = delta
                    }
                    .onEnded { _ in
                        // Reset joystick position and controls except yaw
                        dragLocation = .zero
                        roll = 0.0
                        pitch = 0.0
                    }
            )
            .simultaneousGesture(
                LongPressGesture(minimumDuration: 0.3)
                    .updating($longPressState) { currentState, state, _ in
                        state = currentState
                    }
                    .onChanged { _ in
                        isLongPressing = true
                        initialYawValue = yaw
                    }
                    .onEnded { _ in
                        isLongPressing = false
                    }
            )
        }
        .frame(width: 100, height: 100)
    }
}

// MARK: - Main Turbine Interface

struct BladelessTurbineView: View {

    @StateObject private var controller = TurbineSceneController()

    // Throttle command, 0.0 to 1.0.
    @State private var throttle: Double = 0.6

    // Calculated from the actual configuration.
    @State private var pressureRatio: Double = {
        let config = TurbineConfig()

        return Double(
            config.inletPressure /
            max(1, config.outletPressure)
        )
    }()

    private let designMassFlow: Double = 112.0

    // New state variables for 3D orientation
    @State private var yaw: Double = 0.0
    @State private var pitch: Double = 0.0
    @State private var roll: Double = 0.0

    var body: some View {
        ZStack {
            TurbineSceneView(controller: controller)
                .ignoresSafeArea()

            VStack(alignment: .leading, spacing: 6) {

                // MARK: Top telemetry

                HStack(alignment: .top, spacing: 8) {

                    PressureOverlay(
                        stations: controller.stations,
                        ambient: controller.cfg.outletPressure / 1000.0
                    )

                    Spacer()

                    // MARK: Core HUD

                    VStack(alignment: .leading, spacing: 4) {

                        HStack {
                            Text("CORE HUD")
                                .font(
                                    .system(
                                        size: 10,
                                        weight: .bold,
                                        design: .monospaced
                                    )
                                )
                                .foregroundStyle(.cyan)

                            Spacer()

                            if controller.starterOn {
                                Text("STARTER ACTIVE")
                                    .font(.system(size: 9, weight: .bold))
                                    .foregroundStyle(.orange)
                            }
                        }

                        Divider()

                        LazyVGrid(
                            columns: [
                                GridItem(.flexible()),
                                GridItem(.flexible())
                            ],
                            spacing: 4
                        ) {
                            hudMetric(
                                "SHAFT",
                                String(format: "%.0f RPM", controller.rpm)
                            )

                            hudMetric(
                                "POWER",
                                String(format: "%.0f%%", controller.shaftPercent)
                            )

                            hudMetric(
                                "FLAME",
                                String(format: "%.0f K", controller.flameTemp)
                            )

                            hudMetric(
                                "FUEL",
                                String(format: "%.1f g/s", controller.fuelFlow)
                            )

                            hudMetric(
                                "THRUST",
                                String(format: "%.1f N", controller.thrust)
                            )

                            hudMetric(
                                "EXHAUST",
                                String(format: "%.0f m/s", controller.exhaustVelocity)
                            )

                            hudMetric(
                                "MASS FLOW",
                                String(
                                    format: "%.1f/%.0f kg/s",
                                    controller.massFlow,
                                    designMassFlow
                                )
                            )

                            hudMetric(
                                "PR",
                                String(format: "%.2f×", controller.pressureRatio)
                            )
                        }
                    }
                    .font(.system(size: 9.5, design: .monospaced))
                    .padding(8)
                    .frame(width: 240)
                    .background(
                        .ultraThinMaterial,
                        in: RoundedRectangle(cornerRadius: 8)
                    )
                }

                // MARK: Cockpit controls

                HStack(spacing: 10) {

                    Button("START") {
                        controller.startEngine()
                    }
                    .font(.system(size: 10, weight: .bold))
                    .buttonStyle(.borderedProminent)
                    .tint(.blue)

                    F16ThrottleView(throttle: $throttle)
                        .onChange(of: throttle) { _, newValue in
                            controller.setFuel(Float(newValue))
                        }

                    JoystickView(roll: $roll, pitch: $pitch, yaw: $yaw)

                    Button("RESET") {
                        controller.reset()
                        controller.setFuel(Float(throttle))
                        controller.setPressureRatio(Float(pressureRatio))
                    }
                    .font(.system(size: 10, weight: .bold))
                    .buttonStyle(.bordered)
                    .tint(.red)
                }
                .padding(8)
                .background(
                    .ultraThinMaterial,
                    in: RoundedRectangle(cornerRadius: 8)
                )
            }
            .padding(10)
            .foregroundStyle(.white)
        }
        .onAppear {
            controller.setFuel(Float(throttle))
            controller.setPressureRatio(Float(pressureRatio))
        }
        .onChange(of: yaw) { newYaw in
            controller.setYaw(Float(newYaw))
        }
        .onChange(of: pitch) { newPitch in
            controller.setPitch(Float(newPitch))
        }
        .onChange(of: roll) { newRoll in
            controller.setRoll(Float(newRoll))
        }
    }

    // MARK: HUD Metric

    @ViewBuilder
    private func hudMetric(
        _ label: String,
        _ value: String
    ) -> some View {
        HStack {
            Text(label)
                .foregroundStyle(.secondary)

            Spacer()

            Text(value)
                .fontWeight(.semibold)
        }
    }
}

// MARK: Preview

#Preview {
    BladelessTurbineView()
}

private extension Double {
    func clamped(to limits: ClosedRange<Double>) -> Double {
        min(max(self, limits.lowerBound), limits.upperBound)
    }
}

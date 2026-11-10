//
//  BladelessTurbineView.swift
//  Bladeless Turbine
//

import SwiftUI
import SceneKit

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

    func updateUIView(_ uiView: SCNView, context: Context) {}
}

struct PressureOverlay: View {
    let stations: [PressureStation]
    let ambient: Float

    private func colour(_ kPa: Float) -> Color {
        let ratio = kPa / ambient
        if ratio < 0.97 { return Color(red: 0.4, green: 0.6, blue: 1.0) }
        if ratio < 1.5 { return Color(red: 0.4, green: 0.9, blue: 0.6) }
        if ratio < 3 { return Color(red: 1.0, green: 0.85, blue: 0.3) }
        if ratio < 5 { return Color(red: 1.0, green: 0.55, blue: 0.2) }
        return Color(red: 1.0, green: 0.3, blue: 0.25)
    }

    var body: some View {
        VStack(alignment: .leading, spacing: 4) {
            Text("STATION PRESSURES (kPa)")
                .font(.system(size: 9, weight: .bold, design: .monospaced))
                .foregroundStyle(.cyan)
            ForEach(stations) { s in
                HStack(spacing: 6) {
                    Text(s.name)
                        .frame(width: 115, alignment: .leading)
                    GeometryReader { geo in
                        let frac = CGFloat(min(1, max(0, (s.kPa / ambient) / 7)))
                        ZStack(alignment: .leading) {
                            Capsule().fill(.white.opacity(0.12))
                            Capsule().fill(colour(s.kPa)).frame(width: max(3, geo.size.width * frac))
                        }
                    }
                    .frame(width: 50, height: 6)
                    Text(String(format: "%4.0f", s.kPa))
                        .frame(width: 30, alignment: .trailing)
                    Text(String(format: "%.1f×", s.kPa / ambient))
                        .frame(width: 30, alignment: .trailing)
                        .foregroundStyle(.secondary)
                    Text(String(format: "%4.0fK", s.tempK))
                        .frame(width: 38, alignment: .trailing)
                        .foregroundStyle(.secondary)
                }
                .font(.system(size: 9, design: .monospaced))
            }
        }
        .padding(8)
        .background(.ultraThinMaterial, in: RoundedRectangle(cornerRadius: 8))
    }
}


struct BladelessTurbineView: View {
    @StateObject private var controller = TurbineSceneController()
    @State private var fuel: Double = 0.6
    @State private var pressureRatio: Double = Double(TurbineConfig().defaultPressureRatio)

    var body: some View {
        ZStack {
            TurbineSceneView(controller: controller)
                .ignoresSafeArea()

            VStack(alignment: .leading, spacing: 6) {
                // Top Avionics Header & Pressure Overlay
                HStack(alignment: .top, spacing: 8) {
                    PressureOverlay(stations: controller.stations,
                                    ambient: controller.cfg.ambientPressure)

                    Spacer()

                    // Professional Compact Telemetry HUD Ribbon
                    VStack(alignment: .leading, spacing: 4) {
                        HStack {
                            Text("CORE HUD").font(.system(size: 10, weight: .bold, design: .monospaced)).foregroundStyle(.cyan)
                            Spacer()
                            if controller.starterOn {
                                Text("STARTER ACTIVE").font(.system(size: 9, weight: .bold)).foregroundStyle(.orange)
                            }
                        }
                        Divider()
                        
                        LazyVGrid(columns: [GridItem(.flexible()), GridItem(.flexible())], spacing: 4) {
                            hudMetric("SHAFT", String(format: "%.0f RPM", controller.rpm))
                            hudMetric("POWER", String(format: "%.0f%%", controller.shaftPercent))
                            hudMetric("FLAME", String(format: "%.0f K", controller.flameTemp))
                            hudMetric("FUEL", String(format: "%.1f g/s", controller.fuelFlow))
                            hudMetric("THRUST", String(format: "%.1f N", controller.thrust))
                            hudMetric("EXHAUST", String(format: "%.0f m/s", controller.exhaustVelocity))
                            hudMetric("MASS FLOW", String(format: "%.1f/125kg", controller.massFlow))
                            hudMetric("PR", String(format: "%.1f×", controller.pressureRatio))
                        }
                    }
                    .font(.system(size: 9.5, design: .monospaced))
                    .padding(8)
                    .frame(width: 240)
                    .background(.ultraThinMaterial, in: RoundedRectangle(cornerRadius: 8))
                }

                // Compact Cockpit Control Deck (Throttle)
                HStack(spacing: 10) {
                    Button("START") { controller.startEngine() }
                        .font(.system(size: 10, weight: .bold))
                        .buttonStyle(.borderedProminent)
                        .tint(.blue)

                    F16ThrottleView(throttle: $fuel)
                        .onChange(of: fuel) { _, v in controller.setFuel(Float(v)) }

                    Button("RESET") { controller.reset() }
                        .font(.system(size: 10, weight: .bold))
                        .buttonStyle(.bordered)
                        .tint(.red)
                }
                .padding(8)
                .background(.ultraThinMaterial, in: RoundedRectangle(cornerRadius: 8))

            }
            .padding(10)
            .foregroundStyle(.white)
        }
        .onAppear {
            controller.setFuel(Float(fuel))
            controller.setPressureRatio(Float(pressureRatio))
        }
    }

    @ViewBuilder
    private func hudMetric(_ label: String, _ value: String) -> some View {
        HStack {
            Text(label).foregroundStyle(.secondary)
            Spacer()
            Text(value).fontWeight(.semibold)
        }
    }
}


#Preview { BladelessTurbineView() }

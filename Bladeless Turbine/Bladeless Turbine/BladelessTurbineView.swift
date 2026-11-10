//
//  BladelessTurbineView.swift
//  Bladeless Turbine
//
//  SwiftUI front end: the 3D engine scene, telemetry HUD, pressure overlay,
//  stage counts and engine controls.
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

// MARK: - Pressure Overlay

struct PressureOverlay: View {
    let stations: [PressureStation]
    let ambient: Float

    private func colour(_ kPa: Float) -> Color {
        let ratio = kPa / ambient
        if ratio < 0.97 { return Color(red: 0.4, green: 0.6, blue: 1.0) }       // suction
        if ratio < 1.5 { return Color(red: 0.4, green: 0.9, blue: 0.6) }
        if ratio < 3 { return Color(red: 1.0, green: 0.85, blue: 0.3) }
        if ratio < 5 { return Color(red: 1.0, green: 0.55, blue: 0.2) }
        return Color(red: 1.0, green: 0.3, blue: 0.25)
    }

    var body: some View {
        VStack(alignment: .leading, spacing: 5) {
            Text("GAS PRESSURE  ·  kPa / ratio")
                .font(.system(size: 10, weight: .semibold, design: .monospaced))
                .foregroundStyle(.secondary)
            ForEach(stations) { s in
                HStack(spacing: 6) {
                    Text(s.name)
                        .frame(width: 98, alignment: .leading)
                    GeometryReader { geo in
                        let frac = CGFloat(min(1, max(0, (s.kPa / ambient) / 7)))
                        ZStack(alignment: .leading) {
                            Capsule().fill(.white.opacity(0.12))
                            Capsule().fill(colour(s.kPa)).frame(width: max(3, geo.size.width * frac))
                        }
                    }
                    .frame(width: 80, height: 7)
                    Text(String(format: "%5.0f", s.kPa))
                        .frame(width: 34, alignment: .trailing)
                    Text(String(format: "%.1f×", s.kPa / ambient))
                        .frame(width: 34, alignment: .trailing)
                        .foregroundStyle(.secondary)
                    Text(String(format: "%4.0fK", s.tempK))
                        .frame(width: 42, alignment: .trailing)
                        .foregroundStyle(.secondary)
                }
                .font(.system(size: 10.5, design: .monospaced))
            }
        }
        .padding(10)
        .background(.ultraThinMaterial, in: RoundedRectangle(cornerRadius: 10))
    }
}

// MARK: - Main View

struct BladelessTurbineView: View {
    @StateObject private var controller = TurbineSceneController()
    @State private var fuel: Double = 0.6
    @State private var pressureRatio: Double = Double(TurbineConfig().defaultPressureRatio)

    var body: some View {
        ZStack {
            TurbineSceneView(controller: controller)
                .ignoresSafeArea()

            VStack(alignment: .leading, spacing: 8) {
                // Header Telemetry & Pressure Overlay
                HStack(alignment: .top, spacing: 8) {
                    PressureOverlay(stations: controller.stations,
                                    ambient: controller.cfg.ambientPressure)

                    VStack(alignment: .leading, spacing: 3) {
                        Text("Tesla-Turbine Jet").font(.headline)
                        Text(String(format: "Shaft     %5.0f rpm  (%.0f%%)", controller.rpm, controller.shaftPercent))
                        Text(String(format: "Flame     %5.0f K", controller.flameTemp))
                        Text(String(format: "Fuel      %5.1f g/s", controller.fuelFlow))
                        Text(String(format: "Fuel used %5.1f g", controller.fuelUsed))
                        Text(String(format: "Thrust    %5.1f N", controller.thrust))
                        Text(String(format: "Exhaust   %5.0f m/s", controller.exhaustVelocity))
                        Text(String(format: "PR        %5.1f×", controller.pressureRatio))
                        Text(String(format: "Turbine τ  %4.2f", controller.turbineTorque))
                        Text(String(format: "Compr. τ   %4.2f", controller.compressorLoad))
                        Text("Gas parts  \(controller.activeCount)")
                        Text("Exhausted  \(controller.exhaustedTotal)")
                        if controller.starterOn {
                            Text("STARTER ENGAGED").foregroundStyle(.orange)
                        }
                    }
                    .font(.system(size: 10.5, design: .monospaced))
                    .padding(10)
                    .background(.ultraThinMaterial, in: RoundedRectangle(cornerRadius: 10))
                    Spacer(minLength: 0)
                }

                Spacer()

                // Stage Particle Count Indicator
                HStack(spacing: 8) {
                    ForEach(Array(EngineStage.allCases.enumerated()), id: \.offset) { idx, stage in
                        HStack(spacing: 4) {
                            Circle()
                                .fill(Color(uiColor: stage.color))
                                .frame(width: 8, height: 8)
                            Text("\(stage.title): \(controller.stageCounts.indices.contains(idx) ? controller.stageCounts[idx] : 0)")
                                .font(.system(size: 9, weight: .bold, design: .monospaced))
                        }
                    }
                }
                .padding(8)
                .background(.ultraThinMaterial, in: RoundedRectangle(cornerRadius: 8))

                // Bottom Controls
                VStack(spacing: 8) {
                    HStack(spacing: 12) {
                        Button("Start Engine") { controller.startEngine() }
                            .buttonStyle(.borderedProminent)

                        VStack(alignment: .leading, spacing: 2) {
                            Text("Fuel / Throttle: \(Int(fuel * 100))%")
                                .font(.caption2.monospaced())
                            Slider(value: $fuel, in: 0...1)
                                .onChange(of: fuel) { _, v in controller.setFuel(Float(v)) }
                        }

                        VStack(alignment: .leading, spacing: 2) {
                            Text("Design PR: \(String(format: "%.1f", pressureRatio))×")
                                .font(.caption2.monospaced())
                            Slider(value: $pressureRatio, in: 1.5...8.0, step: 0.5)
                                .onChange(of: pressureRatio) { _, v in controller.setPressureRatio(Float(v)) }
                        }

                        Button("Reset") { controller.reset() }
                            .buttonStyle(.bordered)
                    }
                }
                .padding(12)
                .background(.ultraThinMaterial, in: RoundedRectangle(cornerRadius: 12))
            }
            .padding()
            .foregroundStyle(.white)
        }
        .onAppear {
            controller.setFuel(Float(fuel))
            controller.setPressureRatio(Float(pressureRatio))
        }
    }
}

#Preview { BladelessTurbineView() }

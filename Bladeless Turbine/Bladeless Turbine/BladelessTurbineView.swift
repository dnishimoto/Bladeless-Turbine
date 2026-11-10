//
//  ContentView.swift
//  Bladeless Turbine
//
//  SwiftUI front end: the 3D engine scene plus a live pressure overlay driven by
//  the cellular automaton.
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
        view.isPlaying = true                       // keep the render loop (and CA) running
        view.preferredFramesPerSecond = 60
        view.antialiasingMode = .multisampling4X
        view.backgroundColor = .black
        return view
    }

    func updateUIView(_ uiView: SCNView, context: Context) {}
}

// MARK: - Pressure overlay

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

// MARK: - Stage populations (where the CA's gas is right now)

struct StageBar: View {
    let counts: [Int]

    var body: some View {
        let total = max(1, counts.reduce(0, +))
        VStack(alignment: .leading, spacing: 4) {
            Text("GAS BY STAGE")
                .font(.system(size: 10, weight: .semibold, design: .monospaced))
                .foregroundStyle(.secondary)
            ForEach(EngineStage.allCases) { stage in
                let n = stage.rawValue < counts.count ? counts[stage.rawValue] : 0
                HStack(spacing: 6) {
                    Text(stage.title)
                        .frame(width: 128, alignment: .leading)
                    GeometryReader { geo in
                        ZStack(alignment: .leading) {
                            Capsule().fill(.white.opacity(0.12))
                            Capsule().fill(Color(stage.color))
                                .frame(width: max(2, geo.size.width * CGFloat(n) / CGFloat(total)))
                        }
                    }
                    .frame(width: 70, height: 6)
                    Text("\(n)").frame(width: 34, alignment: .trailing)
                }
                .font(.system(size: 10, design: .monospaced))
            }
        }
        .padding(10)
        .background(.ultraThinMaterial, in: RoundedRectangle(cornerRadius: 10))
    }
}

// MARK: - Main view

struct BladelessTurbineView: View {
    @StateObject private var controller = TurbineSceneController()
    @State private var fuel: Double = 0.6
    @State private var pressureRatio: Double = Double(TurbineConfig().defaultPressureRatio)

    var body: some View {
        ZStack {
            TurbineSceneView(controller: controller)
                .ignoresSafeArea()

            VStack(alignment: .leading, spacing: 8) {
                HStack(alignment: .top, spacing: 8) {
                    VStack(alignment: .leading, spacing: 8) {
                        PressureOverlay(stations: controller.stations,
                                        ambient: controller.cfg.ambientPressure)
                        StageBar(counts: controller.stageCounts)
                    }

                    VStack(alignment: .leading, spacing: 3) {
                        Text("Tesla-Turbine Jet").font(.headline)
                        Text(String(format: "Shaft     %5.0f rpm  (%.0f%%)", controller.rpm, controller.shaftPercent))
                        Text(String(format: "Flame     %5.0f K", controller.flameTemp))
                        Text(String(format: "Press.ratio %4.2f : 1", controller.pressureRatio))
                        Text(String(format: "Fuel      %5.1f g/s", controller.fuelFlow))
                        Text(String(format: "Fuel used %5.1f g", controller.fuelUsed))
                        Text(String(format: "Exit vel. %5.0f m/s", controller.exhaustVelocity))
                        Text(String(format: "Mass flow %5.2f kg/s", controller.massFlow))
                        Text(String(format: "Thrust    %5.0f N", controller.thrust))
                        Text(String(format: "TSFC      %5.0f g/kN·s", controller.tsfc))
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

                HStack(spacing: 12) {
                    Button("Start") { controller.startEngine() }
                        .buttonStyle(.borderedProminent)
                    Text("Fuel")
                    Slider(value: $fuel, in: 0...1)
                        .onChange(of: fuel) { _, v in controller.setFuel(Float(v)) }
                    Text(String(format: "PR %.1f", pressureRatio))
                        .font(.system(size: 12, design: .monospaced))
                    Slider(value: $pressureRatio, in: 2...8)
                        .onChange(of: pressureRatio) { _, v in controller.setPressureRatio(Float(v)) }
                    Button("Reset") { controller.reset() }
                        .buttonStyle(.bordered)
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

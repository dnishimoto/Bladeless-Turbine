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
        view.isPlaying = true                       // keep the render loop (and CA) running
        view.preferredFramesPerSecond = 60
        view.antialiasingMode = .multisampling4X
        view.backgroundColor = .black
        return view
    }

    func updateUIView(_ uiView: SCNView, context: Context) {}
}

struct BladelessTurbineView: View {
    @StateObject private var controller = TurbineSceneController()
    @State private var jet: Double = 0.7

    var body: some View {
        ZStack {
            TurbineSceneView(controller: controller)
                .ignoresSafeArea()

            VStack {
                HStack(alignment: .top) {
                    VStack(alignment: .leading, spacing: 4) {
                        Text("Bladeless Turbine").font(.headline)
                        Text(String(format: "Disk speed  %.0f rpm", controller.rpm))
                        Text("Gas particles  \(controller.activeCount)")
                        Text("Exhausted  \(controller.exhaustedTotal)")
                    }
                    .font(.system(.footnote, design: .monospaced))
                    .padding(10)
                    .background(.ultraThinMaterial, in: RoundedRectangle(cornerRadius: 10))
                    Spacer()
                }
                Spacer()
                HStack(spacing: 12) {
                    Text("Jet")
                    Slider(value: $jet, in: 0...1)
                        .onChange(of: jet) { _, v in controller.setJetIntensity(Float(v)) }
                    Button("Reset") { controller.reset() }
                        .buttonStyle(.borderedProminent)
                }
                .padding(12)
                .background(.ultraThinMaterial, in: RoundedRectangle(cornerRadius: 12))
            }
            .padding()
            .foregroundStyle(.white)
        }
        .onAppear { controller.setJetIntensity(Float(jet)) }
    }
}

#Preview { BladelessTurbineView() }

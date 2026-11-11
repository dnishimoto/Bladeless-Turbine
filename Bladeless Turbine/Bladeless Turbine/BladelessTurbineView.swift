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
        view.allowsCameraControl = false
        view.isPlaying = true
        view.preferredFramesPerSecond = 60
        view.antialiasingMode = .multisampling4X
        view.backgroundColor = .black
        return view
    }

    func updateUIView(_ uiView: SCNView, context: Context) {}
}

/// Compact station strip: one chip per station, laid out in a single row.
struct PressureOverlay: View {
    let stations: [PressureStation]
    let ambient: Float

    private static let shortNames = ["FAN", "LPC", "HPC", "BURN", "HPT", "LPT", "COLL", "DUCT", "NOZ"]

    private func colour(_ kPa: Float) -> Color {
        let ratio = kPa / ambient
        if ratio < 0.97 { return Color(red: 0.4, green: 0.6, blue: 1.0) }
        if ratio < 1.5 { return Color(red: 0.4, green: 0.9, blue: 0.6) }
        if ratio < 3 { return Color(red: 1.0, green: 0.85, blue: 0.3) }
        if ratio < 5 { return Color(red: 1.0, green: 0.55, blue: 0.2) }
        return Color(red: 1.0, green: 0.3, blue: 0.25)
    }

    var body: some View {
        HStack(spacing: 4) {
            ForEach(stations) { s in
                VStack(spacing: 1) {
                    Text("\(s.id + 1) \(s.id < Self.shortNames.count ? Self.shortNames[s.id] : "")")
                        .foregroundStyle(.secondary)
                    Text(String(format: "%.0f kPa", s.kPa))
                        .fontWeight(.semibold)
                        .foregroundStyle(colour(s.kPa))
                    Text(String(format: "%.0f K", s.tempK))
                        .foregroundStyle(.secondary)
                }
                .frame(maxWidth: .infinity)
            }
        }
        .font(.system(size: 8.5, design: .monospaced))
        .padding(.vertical, 4)
        .padding(.horizontal, 6)
        .background(.ultraThinMaterial, in: RoundedRectangle(cornerRadius: 8))
    }
}

struct BladelessTurbineView: View {
    @StateObject private var controller = TurbineSceneController()
    @State private var fuel: Double = 0.6
    @State private var rudder: Double = 0
    @State private var pressureRatio: Double = Double(TurbineConfig().defaultPressureRatio)

    var body: some View {
        ZStack {
            TurbineSceneView(controller: controller)
                .ignoresSafeArea()

            // Overlays hug the top and bottom edges; the middle of the screen stays clear.
            VStack(spacing: 0) {
                topBar
                Spacer(minLength: 0)
                    .allowsHitTesting(false)
                bottomBar
            }
            .padding(.horizontal, 8)
            .padding(.vertical, 4)
            .foregroundStyle(.white)
        }
        .onAppear {
            controller.setFuel(Float(fuel))
            controller.setPressureRatio(Float(pressureRatio))
        }
    }

    // MARK: Top: pressures + telemetry

    private var topBar: some View {
        let c = controller.cycle
        return VStack(spacing: 4) {
            PressureOverlay(stations: controller.stations, ambient: controller.cfg.ambientPressure)
            HStack(spacing: 10) {
                metric("SHAFT", String(format: "%.0f rpm", controller.rpm))
                metric("N", String(format: "%.0f%%", controller.shaftPercent))
                metric("T3", String(format: "%.0f K", controller.flameTemp))
                metric("FUEL", String(format: "%.0f g/s", controller.fuelFlow))
                metric("ṁ AIR", String(format: "%.1f kg/s", c.intakeAirFlow))
                metric("THRUST", String(format: "%.1f kN", controller.thrust / 1000))
                metric("OPR", String(format: "%.1f×", controller.pressureRatio))
                if controller.starterOn {
                    Text("STARTER").font(.system(size: 9, weight: .bold)).foregroundStyle(.orange)
                }
            }
            .padding(.vertical, 3)
            .padding(.horizontal, 8)
            .frame(maxWidth: .infinity)
            .background(.ultraThinMaterial, in: RoundedRectangle(cornerRadius: 8))

            HStack(spacing: 10) {
                metric("ALT", String(format: "%.0f m", controller.flightAltitude))
                metric("AGL", String(format: "%.0f m", controller.flightClearance))
                metric("SPEED", String(format: "%.0f m/s", controller.flightSpeed))
                metric("MACH", String(format: "%.2f", controller.flightMach))
                metric("HDG", String(format: "%.0f°", controller.flightHeading))
                metric("PITCH", String(format: "%+.0f°", controller.flightPitch))
                metric("BANK", String(format: "%+.0f°", controller.flightBank))
                if controller.groundAvoid {
                    Text("PULL UP").font(.system(size: 10, weight: .heavy)).foregroundStyle(.red)
                }
            }
            .padding(.vertical, 3)
            .padding(.horizontal, 8)
            .frame(maxWidth: .infinity)
            .background(.ultraThinMaterial, in: RoundedRectangle(cornerRadius: 8))
        }
    }

    // MARK: Bottom: equations + controls

    private var bottomBar: some View {
        let c = controller.cycle
        return VStack(spacing: 4) {
            HStack(alignment: .top, spacing: 6) {
                eqCard("MASS FLOW  ṁ = A·p₀₃/√T₀₃·F(γ)", [
                    String(format: "air %.1f kg/s  fan M %.2f", c.intakeAirFlow, c.intakeMach),
                    String(format: "f %.4f  ṁe %.1f  CA %.1f kg/s", c.fuelAirRatio, c.exhaustFlow, controller.measuredFlow)
                ])
                eqCard("TORQUE  τ = P / Ω", [
                    String(format: "τt %.0f − τc %.0f − τf %.0f", c.turbineTorque, c.compressorTorque, c.frictionTorque),
                    String(format: "net %+.0f N·m   πc %.1f  πt %.1f", controller.netTorque, c.compressorPR, c.turbinePR)
                ])
                eqCard("FORCE  F = ṁVe + (pe−p0)A", [
                    String(format: "%.1f + %.1f = %.1f kN", c.momentumThrust / 1000, c.pressureThrust / 1000, c.thrust / 1000),
                    String(format: "Ve %.0f m/s  Me %.2f %@", c.exhaustVelocity, c.nozzleExitMach, c.choked ? "choked" : "")
                ])
            }

            HStack(spacing: 10) {
                VStack(spacing: 4) {
                    Button("START") { controller.startEngine() }
                        .font(.system(size: 10, weight: .bold))
                        .buttonStyle(.borderedProminent)
                        .tint(.blue)
                    Button("RESET") { controller.reset() }
                        .font(.system(size: 10, weight: .bold))
                        .buttonStyle(.bordered)
                        .tint(.red)
                }

                VStack(spacing: 2) {
                    HStack {
                        Text("THROTTLE").font(.system(size: 9, weight: .bold, design: .monospaced))
                        Spacer()
                        Text(fuel < 0.05 ? "OFF" : String(format: "%.0f%%", fuel * 100))
                            .font(.system(size: 10, weight: .semibold, design: .monospaced))
                    }
                    Slider(value: $fuel, in: 0...1)
                        .onChange(of: fuel) { _, v in controller.setFuel(Float(v)) }
                    HStack {
                        Text("RUDDER").font(.system(size: 9, weight: .bold, design: .monospaced))
                        Slider(value: $rudder, in: -1...1, onEditingChanged: { editing in
                            if !editing { rudder = 0; controller.setRudder(0) }
                        })
                        .onChange(of: rudder) { _, v in controller.setRudder(Float(v)) }
                    }
                    Button(controller.sideCamera ? "CAMERA: SIDE" : "CAMERA: CHASE") { controller.toggleCamera() }
                        .font(.system(size: 9, weight: .bold))
                        .buttonStyle(.bordered)
                }

                JoystickView { roll, pitch in controller.setStick(roll: roll, pitch: pitch) }
            }
            .padding(.vertical, 4)
            .padding(.horizontal, 8)
            .background(.ultraThinMaterial, in: RoundedRectangle(cornerRadius: 8))
        }
    }

    private func metric(_ label: String, _ value: String) -> some View {
        VStack(spacing: 0) {
            Text(label).font(.system(size: 8, design: .monospaced)).foregroundStyle(.secondary)
            Text(value).font(.system(size: 10, weight: .semibold, design: .monospaced))
        }
        .frame(maxWidth: .infinity)
    }

    private func eqCard(_ title: String, _ lines: [String]) -> some View {
        VStack(alignment: .leading, spacing: 1) {
            Text(title).font(.system(size: 8, weight: .bold, design: .monospaced)).foregroundStyle(.cyan)
            ForEach(lines, id: \.self) { Text($0).font(.system(size: 8.5, design: .monospaced)) }
        }
        .lineLimit(1)
        .minimumScaleFactor(0.6)
        .padding(5)
        .frame(maxWidth: .infinity, alignment: .leading)
        .background(.ultraThinMaterial, in: RoundedRectangle(cornerRadius: 8))
    }
}

/// Virtual flight stick: drag right = roll right, drag up = pitch up (nose up). Springs back to centre.
struct JoystickView: View {
    let onChange: (Float, Float) -> Void
    private let size: CGFloat = 92
    @State private var offset: CGSize = .zero

    var body: some View {
        let radius = size / 2 - 14
        ZStack {
            Circle().fill(.white.opacity(0.10))
            Circle().stroke(.white.opacity(0.35), lineWidth: 1)
            Rectangle().fill(.white.opacity(0.2)).frame(width: 1, height: size - 12)
            Rectangle().fill(.white.opacity(0.2)).frame(width: size - 12, height: 1)
            Circle().fill(.cyan.opacity(0.85)).frame(width: 28, height: 28).offset(offset)
            VStack { Text("PITCH").font(.system(size: 7, design: .monospaced)); Spacer() }
                .padding(.top, 2).foregroundStyle(.secondary)
        }
        .frame(width: size, height: size)
        .contentShape(Circle())
        .gesture(
            DragGesture(minimumDistance: 0)
                .onChanged { g in
                    var dx = g.location.x - size / 2, dy = g.location.y - size / 2
                    let len = (dx * dx + dy * dy).squareRoot()
                    if len > radius { dx *= radius / len; dy *= radius / len }
                    offset = CGSize(width: dx, height: dy)
                    onChange(Float(dx / radius), Float(-dy / radius))
                }
                .onEnded { _ in
                    offset = .zero
                    onChange(0, 0)
                }
        )
    }
}

#Preview { BladelessTurbineView() }

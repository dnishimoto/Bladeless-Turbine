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

        // Drag to swing the orbit camera around / above the engine (orbit mode only).
        let pan = UIPanGestureRecognizer(target: context.coordinator,
                                         action: #selector(Coordinator.pan(_:)))
        pan.maximumNumberOfTouches = 1
        view.addGestureRecognizer(pan)
        return view
    }

    func makeCoordinator() -> Coordinator { Coordinator(controller: controller) }

    final class Coordinator: NSObject {
        let controller: TurbineSceneController
        init(controller: TurbineSceneController) { self.controller = controller }

        @objc func pan(_ g: UIPanGestureRecognizer) {
            let t = g.translation(in: g.view)
            g.setTranslation(.zero, in: g.view)
            controller.orbitDrag(dx: Float(t.x), dy: Float(t.y))
        }
    }

    func updateUIView(_ uiView: SCNView, context: Context) {}
}

/// Compact station strip with vertical pressure bar charts.
struct PressureOverlay: View {
    let stations: [PressureStation]
    let ambient: Float

    static let shortNames = ["FAN", "LPC", "HPC", "BURN", "HPT", "LPT", "COLL", "DUCT", "NOZ"]

    private func colour(_ kPa: Float) -> Color {
        let ratio = kPa / ambient
        if ratio < 0.97 { return Color(red: 0.4, green: 0.6, blue: 1.0) }
        if ratio < 1.5 { return Color(red: 0.4, green: 0.9, blue: 0.6) }
        if ratio < 3 { return Color(red: 1.0, green: 0.85, blue: 0.3) }
        if ratio < 5 { return Color(red: 1.0, green: 0.55, blue: 0.2) }
        return Color(red: 1.0, green: 0.3, blue: 0.25)
    }

    var body: some View {
        let maxKPa = stations.map { $0.kPa }.max() ?? 200_000

        return HStack(spacing: 4) {
            ForEach(stations) { s in
                VStack(spacing: 2) {
                    // Vertical pressure bar indicator
                    ZStack(alignment: .bottom) {
                        RoundedRectangle(cornerRadius: 2)
                            .fill(Color.white.opacity(0.1))
                            .frame(width: 8, height: 36)

                        RoundedRectangle(cornerRadius: 2)
                            .fill(colour(s.kPa))
                            .frame(
                                width: 8,
                                height: max(4, CGFloat(min(1.0, s.kPa / maxKPa)) * 36)
                            )
                    }
                    .padding(.bottom, 2)

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
        .padding(.vertical, 6)
        .padding(.horizontal, 6)
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

            VStack(spacing: 0) {
                topBar
                Spacer(minLength: 0)
                    .allowsHitTesting(false)
                bottomBar
            }
            .padding(.horizontal, 8)
            .padding(.vertical, 4)
            .foregroundStyle(.white)

            // Standalone Single Score Overlay Badge (Top-Right)
            VStack {
                HStack {
                    Spacer()
                    VStack(spacing: 1) {
                        Text("SCORE")
                            .font(.system(size: 9, weight: .bold, design: .monospaced))
                            .foregroundStyle(.yellow)
                        Text("\(controller.score)")
                            .font(.system(size: 18, weight: .heavy, design: .monospaced))
                            .foregroundStyle(.white)
                            .contentTransition(.numericText())
                    }
                    .padding(.vertical, 6)
                    .padding(.horizontal, 12)
                    .background(.ultraThinMaterial, in: RoundedRectangle(cornerRadius: 10))
                    .overlay(
                        RoundedRectangle(cornerRadius: 10)
                            .stroke(Color.yellow.opacity(0.4), lineWidth: 1)
                    )
                    .padding(.top, 50)
                    .padding(.trailing, 12)
                }
                Spacer()
            }
            .allowsHitTesting(false)
        }
        .onAppear {
            controller.setFuel(Float(fuel))
            controller.setPressureRatio(Float(pressureRatio))
        }
    }

    private var topBar: some View {
            let c = controller.cycle
            return VStack(spacing: 4) {
                // Streamlined key indicators row
                HStack(spacing: 10) {
                    metric("FUEL RATE", String(format: "%.0f g/s", controller.fuelFlow))
                    metric("SHAFT", String(format: "%.0f rpm", controller.rpm))
                    metric("THROTTLE", String(format: "%.0f%%", fuel * 100))
                    metric("AIR INTAKE", String(format: "%.1f kg/s", c.intakeAirFlow))
                    metric("THRUST", String(format: "%.1f kN", controller.thrust / 1000))
                    metric("MASS FLOW", String(format: "%.1f kg/s", controller.massFlow))
                }
                .padding(.vertical, 3)
                .padding(.horizontal, 8)
                .frame(maxWidth: .infinity)
                .background(.ultraThinMaterial, in: RoundedRectangle(cornerRadius: 8))

                HStack(spacing: 10) {
                    metric("ALT", String(format: "%.0f m", controller.flightAltitude))
                    metric("SPEED", String(format: "%.0f m/s", controller.flightSpeed))
                    metric("MACH", String(format: "%.2f", controller.flightMach))
                    if controller.groundAvoid {
                        Text("PULL UP").font(.system(size: 10, weight: .heavy)).foregroundStyle(.red)
                    }
                }
                .padding(.vertical, 3)
                .padding(.horizontal, 8)
                .frame(maxWidth: .infinity)
                .background(.ultraThinMaterial, in: RoundedRectangle(cornerRadius: 8))

                HStack(alignment: .top, spacing: 6) {
                    metric("MASS FLOW",
                           String(format:" %.1f kg/s",  controller.measuredFlow))
                    metric("ALT", String(format: "%.0f m", controller.flightAltitude))
                    metric("SPEED", String(format: "%.0f m/s", controller.flightSpeed))
                    metric("MACH", String(format: "%.2f", controller.flightMach))
       
                    if controller.groundAvoid {
                        Text("PULL UP").font(.system(size: 10, weight: .heavy)).foregroundStyle(.red)
                    }
                }
            }
        }

    var bottomBar: some View {
        VStack(spacing: 4) {
            HStack(spacing: 10) {
                F16ThrottleView(throttle: $fuel)
                    .onChange(of: fuel) { _, v in controller.setFuel(Float(v)) }

                Button {
                    controller.toggleCamera()
                } label: {
                    Image(systemName: controller.sideCamera ? "globe" : "globe.americas")
                        .font(.system(size: 17, weight: .bold))
                        .frame(width: 36, height: 36)
                }
                .buttonStyle(.bordered)
                .accessibilityLabel(
                    controller.sideCamera
                        ? "Switch to chase camera"
                        : "Switch to orbit camera"
                )
                .accessibilityValue(
                    controller.sideCamera
                        ? "Orbit \(controller.orbitPercent) percent"
                        : "Chase camera"
                )

                AttitudeIndicator(bank: controller.flightBank, pitch: controller.flightPitch)

                JoystickView { roll, pitch in controller.setStick(roll: roll, pitch: pitch) }
            }
            .padding(.vertical, 4)
            .padding(.horizontal, 8)
            .background(.ultraThinMaterial, in: RoundedRectangle(cornerRadius: 8))
        }
    }

    func metric(_ label: String, _ value: String) -> some View {
        VStack(spacing: 0) {
            Text(label).font(.system(size: 10, design: .monospaced)).foregroundStyle(.secondary)
            Text(value).font(.system(size: 11, weight: .semibold, design: .monospaced))
        }
        .frame(maxWidth: .infinity)
    }

    func eqCard(_ title: String, _ lines: [String]) -> some View {
        VStack(alignment: .leading, spacing: 3) {
            Text(title)
                .font(.system(size: 10, weight: .bold, design: .monospaced))
                .foregroundStyle(.cyan)

            ForEach(lines, id: \.self) { line in
                Text(line)
                    .font(.system(size: 10.5, design: .monospaced))
            }
        }
        .lineLimit(1)
        .minimumScaleFactor(0.6)
        .padding(6)
        .frame(maxWidth: .infinity, alignment: .leading)
        .background(
            .ultraThinMaterial,
            in: RoundedRectangle(cornerRadius: 8)
        )
    }
}

#Preview { BladelessTurbineView() }

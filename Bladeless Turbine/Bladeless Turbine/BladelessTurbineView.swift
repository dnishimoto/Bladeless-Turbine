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

/// Compact station strip with vertical pressure bar charts.
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
        }
        .onAppear {
            controller.setFuel(Float(fuel))
            controller.setPressureRatio(Float(pressureRatio))
        }
    }

    private var topBar: some View {
        let c = controller.cycle
        return VStack(spacing: 4) {
            PressureOverlay(stations: controller.stations, ambient: controller.cfg.ambientPressure)

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

            HStack(alignment: .top, spacing: 6) {
                eqCard("MASS FLOW", [
                    String(format: "f %.4f  ṁe %.1f  CA %.1f kg/s", c.fuelAirRatio, c.exhaustFlow, controller.measuredFlow)
                ])
            }
        }
    }

    private var bottomBar: some View {
        VStack(spacing: 4) {
            HStack(spacing: 10) {
                F16ThrottleView(throttle: $fuel)
                    .onChange(of: fuel) { _, v in controller.setFuel(Float(v)) }

                Button(controller.sideCamera ? "CAMERA\nORBIT" : "CAMERA\nCHASE") { controller.toggleCamera() }
                    .font(.system(size: 9, weight: .bold))
                    .multilineTextAlignment(.center)
                    .buttonStyle(.bordered)

                AttitudeIndicator(bank: controller.flightBank, pitch: controller.flightPitch)

                JoystickView { roll, pitch in controller.setStick(roll: roll, pitch: pitch) }
            }
            .padding(.vertical, 4)
            .padding(.horizontal, 8)
            .background(.ultraThinMaterial, in: RoundedRectangle(cornerRadius: 8))
        }
    }

    private func metric(_ label: String, _ value: String) -> some View {
        VStack(spacing: 0) {
            Text(label).font(.system(size: 10, design: .monospaced)).foregroundStyle(.secondary)
            Text(value).font(.system(size: 11, weight: .semibold, design: .monospaced))
        }
        .frame(maxWidth: .infinity)
    }

    private func eqCard(_ title: String, _ lines: [String]) -> some View {
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

/// One flight stick: drag left/right to bank and turn, up/down to pitch. Springs back to centre.
struct JoystickView: View {
    let onChange: (Float, Float) -> Void
    private let size: CGFloat = 110
    @State private var offset: CGSize = .zero

    var body: some View {
        let radius = size / 2 - 16
        ZStack {
            Circle().fill(.white.opacity(0.10))
            Circle().stroke(.white.opacity(0.35), lineWidth: 1)
            Rectangle().fill(.white.opacity(0.2)).frame(width: 1, height: size - 12)
            Rectangle().fill(.white.opacity(0.2)).frame(width: size - 12, height: 1)
            Image(systemName: "arrow.up").font(.system(size: 9)).offset(y: -size / 2 + 8)
            Image(systemName: "arrow.down").font(.system(size: 9)).offset(y: size / 2 - 8)
            Image(systemName: "arrow.left").font(.system(size: 9)).offset(x: -size / 2 + 8)
            Image(systemName: "arrow.right").font(.system(size: 9)).offset(x: size / 2 - 8)
            Circle().fill(.cyan.opacity(0.85)).frame(width: 32, height: 32).offset(offset)
        }
        .foregroundStyle(.secondary)
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
                .offset(y: CGFloat(pitch) * pixelsPerDegree)
                .rotationEffect(.degrees(Double(-bank)))

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

#Preview { BladelessTurbineView() }

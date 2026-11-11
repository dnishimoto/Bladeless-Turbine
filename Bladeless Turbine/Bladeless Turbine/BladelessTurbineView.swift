//
//  BladelessTurbineView.swift
//  Bladeless Turbine
//

import Foundation
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

    func updateUIView(
        _ uiView: SCNView,
        context: Context
    ) {
        // The controller handles ongoing SceneKit updates.
    }
}

// MARK: - Pressure Overlay

struct PressureOverlay: View {
    let stations: [PressureStation]
    let ambient: Float

    var body: some View {
        VStack(alignment: .leading, spacing: 4) {
            title

            ForEach(stations) { station in
                StationPressureRow(
                    name: station.name,
                    pressureKPa: Float(station.kPa),
                    temperatureK: Float(station.tempK),
                    ambientKPa: ambient
                )
            }
        }
        .padding(8)
        .background(
            .ultraThinMaterial,
            in: RoundedRectangle(cornerRadius: 8)
        )
    }

    private var title: some View {
        Text("STATION PRESSURES (kPa)")
            .font(
                .system(
                    size: 9,
                    weight: .bold,
                    design: .monospaced
                )
            )
            .foregroundStyle(.cyan)
    }
}

// MARK: - Pressure Station Row

private struct StationPressureRow: View {
    let name: String
    let pressureKPa: Float
    let temperatureK: Float
    let ambientKPa: Float

    private var pressureRatio: Double {
        let safeAmbient: Float = max(0.001, ambientKPa)
        return Double(pressureKPa / safeAmbient)
    }

    private var fillFraction: CGFloat {
        let fraction = pressureRatio / 7.0
        let boundedFraction = min(1.0, max(0.0, fraction))

        return CGFloat(boundedFraction)
    }

    private var barWidth: CGFloat {
        max(3.0, 50.0 * fillFraction)
    }

    private var pressureText: String {
        String(format: "%4.0f", Double(pressureKPa))
    }

    private var ratioText: String {
        String(format: "%.1f×", pressureRatio)
    }

    private var temperatureText: String {
        String(format: "%4.0fK", Double(temperatureK))
    }

    private var pressureColor: Color {
        let ratio = pressureRatio

        if ratio < 0.97 {
            return Color(
                red: 0.4,
                green: 0.6,
                blue: 1.0
            )
        }

        if ratio < 1.5 {
            return Color(
                red: 0.4,
                green: 0.9,
                blue: 0.6
            )
        }

        if ratio < 3.0 {
            return Color(
                red: 1.0,
                green: 0.85,
                blue: 0.3
            )
        }

        if ratio < 5.0 {
            return Color(
                red: 1.0,
                green: 0.55,
                blue: 0.2
            )
        }

        return Color(
            red: 1.0,
            green: 0.3,
            blue: 0.25
        )
    }

    var body: some View {
        HStack(spacing: 6) {
            Text(name)
                .frame(width: 115, alignment: .leading)

            pressureBar

            Text(pressureText)
                .frame(width: 30, alignment: .trailing)

            Text(ratioText)
                .frame(width: 30, alignment: .trailing)
                .foregroundStyle(.secondary)

            Text(temperatureText)
                .frame(width: 38, alignment: .trailing)
                .foregroundStyle(.secondary)
        }
        .font(.system(size: 9, design: .monospaced))
    }

    private var pressureBar: some View {
        ZStack(alignment: .leading) {
            Capsule()
                .fill(Color.white.opacity(0.12))

            Capsule()
                .fill(pressureColor)
                .frame(width: barWidth)
        }
        .frame(width: 50, height: 6)
    }
}

// MARK: - Joystick

struct JoystickView: View {
    @Binding var roll: Double
    @Binding var pitch: Double
    @Binding var yaw: Double

    @State private var dragLocation: CGPoint = .zero
    @State private var yawControlActive = false
    @State private var initialYawValue: Double = 0
    @State private var latestDragHeight: CGFloat = 0
    @State private var yawStartDragHeight: CGFloat = 0

    private let controlSize: CGFloat = 100
    private let knobSize: CGFloat = 40

    private var radius: CGFloat {
        controlSize / 2.0
    }

    private var travelRadius: CGFloat {
        (controlSize - knobSize) / 2.0
    }

    private var knobColor: Color {
        if yawControlActive {
            return Color.orange.opacity(0.8)
        }

        return Color.cyan.opacity(0.7)
    }

    var body: some View {
        ZStack {
            Circle()
                .fill(Color.gray.opacity(0.2))

            Circle()
                .stroke(
                    Color.white.opacity(0.4),
                    lineWidth: 2
                )

            knob
        }
        .frame(width: controlSize, height: controlSize)
        .contentShape(Circle())
        .gesture(dragGesture)
        .simultaneousGesture(yawActivationGesture)
    }

    private var knob: some View {
        Circle()
            .fill(knobColor)
            .frame(width: knobSize, height: knobSize)
            .offset(
                x: dragLocation.x,
                y: dragLocation.y
            )
            .animation(
                .easeOut(duration: 0.1),
                value: dragLocation
            )
    }

    private var dragGesture: some Gesture {
        DragGesture(minimumDistance: 0)
            .onChanged { value in
                updateDrag(value)
            }
            .onEnded { _ in
                finishDrag()
            }
    }

    private var yawActivationGesture: some Gesture {
        LongPressGesture(
            minimumDuration: 0.3,
            maximumDistance: 100
        )
        .onEnded { recognized in
            guard recognized else { return }

            initialYawValue = yaw
            yawStartDragHeight = latestDragHeight
            yawControlActive = true
        }
    }

    private func updateDrag(_ value: DragGesture.Value) {
        latestDragHeight = value.translation.height

        var deltaX = value.location.x - radius
        var deltaY = value.location.y - radius

        let distanceSquared =
            deltaX * deltaX + deltaY * deltaY

        let distance = sqrt(distanceSquared)

        if distance > travelRadius {
            let scale = travelRadius / distance

            deltaX *= scale
            deltaY *= scale
        }

        roll = Double(deltaX / travelRadius)
        pitch = -Double(deltaY / travelRadius)

        if yawControlActive {
            let relativeDrag =
                value.translation.height - yawStartDragHeight

            let yawChange = Double(relativeDrag / radius)
            let requestedYaw = initialYawValue - yawChange

            yaw = min(1.0, max(-1.0, requestedYaw))
        }

        dragLocation = CGPoint(
            x: deltaX,
            y: deltaY
        )
    }

    private func finishDrag() {
        dragLocation = .zero

        roll = 0
        pitch = 0

        // Preserve the selected yaw after release.
        yawControlActive = false
        latestDragHeight = 0
        yawStartDragHeight = 0
    }
}

// MARK: - HUD Data

private struct TurbineHUDData {
    let starterOn: Bool
    let rpm: Double
    let shaftPercent: Double
    let flameTemp: Double
    let fuelFlow: Double
    let thrust: Double
    let exhaustVelocity: Double
    let massFlow: Double
    let designMassFlow: Double
    let pressureRatio: Double
}

// MARK: - HUD Metric

private struct TurbineHUDMetric: View {
    let label: String
    let value: String

    var body: some View {
        HStack(spacing: 4) {
            Text(label)
                .foregroundStyle(.secondary)
                .lineLimit(1)

            Spacer(minLength: 2)

            Text(value)
                .fontWeight(.semibold)
                .lineLimit(1)
                .minimumScaleFactor(0.65)
        }
        .frame(
            maxWidth: .infinity,
            alignment: .leading
        )
    }
}

// MARK: - Core HUD

private struct TurbineCoreHUD: View {
    let data: TurbineHUDData

    private let columns: [GridItem] = [
        GridItem(.flexible(), alignment: .leading),
        GridItem(.flexible(), alignment: .leading)
    ]

    var body: some View {
        VStack(alignment: .leading, spacing: 4) {
            header

            Divider()

            metrics
        }
        .font(.system(size: 9.5, design: .monospaced))
        .padding(8)
        .frame(width: 240)
        .background(
            .ultraThinMaterial,
            in: RoundedRectangle(cornerRadius: 8)
        )
    }

    private var header: some View {
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

            if data.starterOn {
                Text("STARTER ACTIVE")
                    .font(.system(size: 9, weight: .bold))
                    .foregroundStyle(.orange)
            }
        }
    }

    private var metrics: some View {
        LazyVGrid(
            columns: columns,
            alignment: .leading,
            spacing: 4
        ) {
            TurbineHUDMetric(
                label: "SHAFT",
                value: shaftText
            )

            TurbineHUDMetric(
                label: "POWER",
                value: powerText
            )

            TurbineHUDMetric(
                label: "FLAME",
                value: flameText
            )

            TurbineHUDMetric(
                label: "FUEL",
                value: fuelText
            )

            TurbineHUDMetric(
                label: "THRUST",
                value: thrustText
            )

            TurbineHUDMetric(
                label: "EXHAUST",
                value: exhaustText
            )

            TurbineHUDMetric(
                label: "MASS FLOW",
                value: massFlowText
            )

            TurbineHUDMetric(
                label: "PR",
                value: pressureRatioText
            )
        }
    }

    private var shaftText: String {
        String(format: "%.0f RPM", data.rpm)
    }

    private var powerText: String {
        String(format: "%.0f%%", data.shaftPercent)
    }

    private var flameText: String {
        String(format: "%.0f K", data.flameTemp)
    }

    private var fuelText: String {
        String(format: "%.1f g/s", data.fuelFlow)
    }

    private var thrustText: String {
        String(format: "%.1f N", data.thrust)
    }

    private var exhaustText: String {
        String(format: "%.0f m/s", data.exhaustVelocity)
    }

    private var massFlowText: String {
        String(
            format: "%.1f/%.0f kg/s",
            data.massFlow,
            data.designMassFlow
        )
    }

    private var pressureRatioText: String {
        String(format: "%.2f×", data.pressureRatio)
    }
}

// MARK: - Cockpit Controls

private struct TurbineCockpitControls: View {
    @Binding var throttle: Double
    @Binding var roll: Double
    @Binding var pitch: Double
    @Binding var yaw: Double

    let onStart: () -> Void
    let onReset: () -> Void

    var body: some View {
        HStack(spacing: 10) {
            startButton
            throttleControl
            joystickControl
            resetButton
        }
        .padding(8)
        .background(
            .ultraThinMaterial,
            in: RoundedRectangle(cornerRadius: 8)
        )
    }

    private var startButton: some View {
        Button(action: onStart) {
            Text("START")
                .font(.system(size: 10, weight: .bold))
        }
        .buttonStyle(.borderedProminent)
        .tint(.blue)
    }

    private var throttleControl: some View {
        F16ThrottleView(throttle: $throttle)
    }

    private var joystickControl: some View {
        JoystickView(
            roll: $roll,
            pitch: $pitch,
            yaw: $yaw
        )
    }

    private var resetButton: some View {
        Button("RESET", action: onReset)
            .font(.system(size: 10, weight: .bold))
            .buttonStyle(.bordered)
            .tint(.red)
    }
}

// MARK: - Main Turbine Interface

struct BladelessTurbineView: View {
    @StateObject private var controller = TurbineSceneController()

    // Throttle command: 0.0 to 1.0.
    @State private var throttle: Double = 0.6

    @State private var pressureRatio: Double =
        BladelessTurbineView.defaultPressureRatio

    // Local joystick values.
    // These are not connected to controller orientation methods.
    @State private var yaw: Double = 0
    @State private var pitch: Double = 0
    @State private var roll: Double = 0

    private let designMassFlow: Double = 112.0

    private static var defaultPressureRatio: Double {
        let config = TurbineConfig()

        let inlet = Double(config.inletPressure)
        let outlet = Double(config.outletPressure)
        let safeOutlet = max(1.0, outlet)

        return inlet / safeOutlet
    }

    var body: some View {
        sceneWithOverlay
            .onAppear {
                applyControlSettings()
            }
            .onChange(of: throttle) { _, newValue in
                controller.setFuel(Float(newValue))
            }
    }

    // MARK: - Scene Layout

    private var sceneWithOverlay: some View {
        ZStack(alignment: .topLeading) {
            turbineScene
            instrumentOverlay
        }
    }

    private var turbineScene: some View {
        TurbineSceneView(controller: controller)
            .ignoresSafeArea()
    }

    private var instrumentOverlay: some View {
        VStack(alignment: .leading, spacing: 6) {
            telemetryRow
            cockpitControls
        }
        .padding(10)
        .foregroundStyle(.white)
    }

    // MARK: - Telemetry

    private var telemetryRow: some View {
        HStack(alignment: .top, spacing: 8) {
            pressureOverlay

            Spacer(minLength: 8)

            coreHUD
        }
    }

    private var pressureOverlay: some View {
        PressureOverlay(
            stations: controller.stations,
            ambient: ambientPressureKPa
        )
    }

    private var ambientPressureKPa: Float {
        Float(controller.cfg.outletPressure) / 1000.0
    }

    private var coreHUD: some View {
        TurbineCoreHUD(data: hudData)
    }

    private var hudData: TurbineHUDData {
        let starterOn = controller.starterOn
        let rpm = Double(controller.rpm)
        let shaftPercent = Double(controller.shaftPercent)
        let flameTemp = Double(controller.flameTemp)
        let fuelFlow = Double(controller.fuelFlow)
        let thrust = Double(controller.thrust)
        let exhaustVelocity = Double(controller.exhaustVelocity)
        let massFlow = Double(controller.massFlow)
        let currentPressureRatio = Double(controller.pressureRatio)

        return TurbineHUDData(
            starterOn: starterOn,
            rpm: rpm,
            shaftPercent: shaftPercent,
            flameTemp: flameTemp,
            fuelFlow: fuelFlow,
            thrust: thrust,
            exhaustVelocity: exhaustVelocity,
            massFlow: massFlow,
            designMassFlow: designMassFlow,
            pressureRatio: currentPressureRatio
        )
    }

    // MARK: - Controls

    private var cockpitControls: some View {
        TurbineCockpitControls(
            throttle: $throttle,
            roll: $roll,
            pitch: $pitch,
            yaw: $yaw,
            onStart: startEngine,
            onReset: resetEngine
        )
    }

    // MARK: - Controller Actions

    private func startEngine() {
        controller.startEngine()
    }

    private func resetEngine() {
        controller.reset()

        // Preserve the selected throttle and pressure ratio.
        applyControlSettings()

        // Reset local joystick state.
        yaw = 0
        pitch = 0
        roll = 0
    }

    private func applyControlSettings() {
        controller.setFuel(Float(throttle))
        controller.setPressureRatio(Float(pressureRatio))
    }
}

// MARK: - Preview

#Preview {
    BladelessTurbineView()
}

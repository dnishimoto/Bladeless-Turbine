

import Foundation
import AVFoundation

final class EngineAudio {
    private let engine = AVAudioEngine()
    private var source: AVAudioSourceNode?
    private var sampleRate: Double = 44_100

    // Written from the main/render thread, read on the audio thread (plain floats are fine here).
    private var shaft: Float = 0.3
    private var roar: Float = 0
    private var burner: Float = 0
    private var gain: Float = 0.22
    private(set) var isOn = false

    // Audio-thread state
    private var phaseA: Float = 0, phaseB: Float = 0
    private var lowA: Float = 0, lowB: Float = 0, lowC: Float = 0
    private var seed: UInt32 = 0x1234_5678

    init() {
        let format = engine.outputNode.inputFormat(forBus: 0)
        sampleRate = format.sampleRate > 0 ? format.sampleRate : 44_100
        let mono = AVAudioFormat(standardFormatWithSampleRate: sampleRate, channels: 1)

        let node = AVAudioSourceNode { [unowned self] _, _, frameCount, audioBufferList -> OSStatus in
            let buffers = UnsafeMutableAudioBufferListPointer(audioBufferList)
            let sr = Float(self.sampleRate)
            let rpm = self.shaft, roar = self.roar, burner = self.burner, gain = self.gain
            let whineHz = 500 + 2600 * rpm
            let humHz = 90 + 240 * rpm

            for frame in 0..<Int(frameCount) {
                self.phaseA += 2 * .pi * whineHz / sr
                self.phaseB += 2 * .pi * humHz / sr
                if self.phaseA > 2 * .pi { self.phaseA -= 2 * .pi }
                if self.phaseB > 2 * .pi { self.phaseB -= 2 * .pi }

                // white noise
                self.seed = self.seed &* 1_664_525 &+ 1_013_904_223
                let white = Float(Int32(bitPattern: self.seed)) / Float(Int32.max)

                // low-pass filtered noise at three cut-offs: hiss, roar, rumble
                self.lowA += 0.35 * (white - self.lowA)
                self.lowB += 0.05 * (white - self.lowB)
                self.lowC += 0.008 * (white - self.lowC)

                let whine = (sin(self.phaseA) + 0.4 * sin(2 * self.phaseA)) * (0.05 + 0.10 * rpm)
                let hum = sin(self.phaseB) * 0.08 * rpm
                let jet = self.lowA * (0.10 + 0.45 * roar) + self.lowB * (0.2 + 0.9 * roar)
                let rumble = self.lowC * (1.5 + 6.0 * burner)

                let sample = max(-1, min(1, (whine + hum + jet + rumble) * gain))
                for buffer in buffers {
                    buffer.mData?.assumingMemoryBound(to: Float.self)[frame] = sample
                }
            }
            return noErr
        }
        source = node
        engine.attach(node)
        engine.connect(node, to: engine.mainMixerNode, format: mono)
    }

    func start() {
        guard !isOn else { return }
        do {
            try AVAudioSession.sharedInstance().setCategory(.ambient, mode: .default, options: [.mixWithOthers])
            try AVAudioSession.sharedInstance().setActive(true)
            try engine.start()
            isOn = true
        } catch {
            isOn = false
        }
    }

    func stop() {
        engine.pause()
        isOn = false
    }

    /// shaftFraction 0…1+, thrustFraction 0…1, mach ≥ 0, afterburner 0…1
    func update(shaftFraction: Float, thrustFraction: Float, mach: Float, afterburner: Float) {
        shaft = max(0, min(1.1, shaftFraction))
        roar = max(0, min(1, 0.7 * thrustFraction + 0.3 * min(1, mach)))
        burner = max(0, min(1, afterburner))
    }
}

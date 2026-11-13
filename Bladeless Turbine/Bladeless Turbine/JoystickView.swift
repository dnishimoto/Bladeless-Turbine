//
//  File.swift
//  Bladeless Turbine
//
//  Created by David Nishimoto on 11/12/26.
//

import Foundation
import SwiftUI
import SceneKit

/// One flight stick: drag left/right to bank and turn, pull down to pitch up. Springs back to centre.
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
                    var dx = g.location.x - size / 2
                    var dy = g.location.y - size / 2
                    let len = (dx * dx + dy * dy).squareRoot()
                    if len > radius { dx *= radius / len; dy *= radius / len }
                    offset = CGSize(width: dx, height: dy)
                    onChange(Float(dx / radius), Float(dy / radius))
                }
                .onEnded { _ in
                    offset = .zero
                    onChange(0, 0)
                }
        )
    }
}

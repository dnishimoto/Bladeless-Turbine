//
//  Terrain.swift
//  Bladeless Turbine
//
//  Created by David Nishimoto on 11/11/26.
//

import Foundation
import SceneKit

enum Terrain {
    /// Deterministic height field in metres (valleys, ridges, flat lakes at 0 m).
    static func height(_ x: Float, _ z: Float) -> Float {
        var h: Float = 230
        h += 220 * sin(x * 0.0009) * sin(z * 0.0011)
        h += 120 * sin(x * 0.0023 + 1.7) * cos(z * 0.0019)
        h += 45 * sin(x * 0.0061) * sin(z * 0.0053 + 0.5)
        h += 12 * sin(x * 0.017 + z * 0.013)
        return max(0, h)
    }

    static let cell: Float = 60
    static let cells = 128

    private static func color(for h: Float, slope: Float) -> (Float, Float, Float) {
        if h <= 0.5 { return (0.12, 0.28, 0.55) }                       // water
        if h < 60 { return (0.76, 0.70, 0.50) }                         // shore
        if h < 260 { return slope > 0.45 ? (0.45, 0.40, 0.33) : (0.20, 0.45, 0.20) }
        if h < 420 { return slope > 0.40 ? (0.48, 0.42, 0.36) : (0.40, 0.45, 0.25) }
        return (0.92, 0.94, 0.96)                                        // snow
    }

    /// Mesh centred on (cx, cz), snapped to the grid so heights never change between rebuilds.
    static func mesh(centerX: Float, centerZ: Float) -> SCNGeometry {
        let n = cells + 1
        let ox = (centerX / cell).rounded() * cell - Float(cells) / 2 * cell
        let oz = (centerZ / cell).rounded() * cell - Float(cells) / 2 * cell

        var verts = [SCNVector3](); verts.reserveCapacity(n * n)
        var norms = [SCNVector3](); norms.reserveCapacity(n * n)
        var colors = [Float](); colors.reserveCapacity(n * n * 4)
        for j in 0..<n {
            for i in 0..<n {
                let x = ox + Float(i) * cell, z = oz + Float(j) * cell
                let h = height(x, z)
                let dx = (height(x + cell, z) - height(x - cell, z)) / (2 * cell)
                let dz = (height(x, z + cell) - height(x, z - cell)) / (2 * cell)
                let nv = simd_normalize(SIMD3<Float>(-dx, 1, -dz))
                verts.append(SCNVector3(x, h, z))
                norms.append(SCNVector3(nv.x, nv.y, nv.z))
                let c = color(for: h, slope: (dx * dx + dz * dz).squareRoot())
                colors.append(contentsOf: [c.0, c.1, c.2, 1])
            }
        }
        var idx = [Int32](); idx.reserveCapacity(cells * cells * 6)
        for j in 0..<cells {
            for i in 0..<cells {
                let a = Int32(j * n + i), b = a + 1, c = a + Int32(n), d = c + 1
                idx.append(contentsOf: [a, c, b, b, c, d])
            }
        }
        let colorData = colors.withUnsafeBufferPointer { Data(buffer: $0) }
        let colorSource = SCNGeometrySource(data: colorData, semantic: .color, vectorCount: n * n,
                                            usesFloatComponents: true, componentsPerVector: 4,
                                            bytesPerComponent: MemoryLayout<Float>.size,
                                            dataOffset: 0, dataStride: MemoryLayout<Float>.size * 4)
        let element = SCNGeometryElement(indices: idx, primitiveType: .triangles)
        let geo = SCNGeometry(sources: [SCNGeometrySource(vertices: verts), SCNGeometrySource(normals: norms), colorSource],
                              elements: [element])
        let m = SCNMaterial()
        m.lightingModel = .lambert
        m.diffuse.contents = UIColor.white
        m.isDoubleSided = true
        geo.materials = [m]
        return geo
    }
}

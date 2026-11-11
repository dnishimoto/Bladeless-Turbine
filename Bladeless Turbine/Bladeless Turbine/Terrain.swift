
import Foundation
import SceneKit
import UIKit
import simd

enum Terrain {

    // MARK: - Mesh Configuration

    static let cell: Float = 60.0
    static let cells: Int = 128

    // MARK: - Fractal Noise

    private static func hash(_ x: Float, _ z: Float) -> Float {
        let value = sin(x * 127.1 + z * 311.7) * 43758.5453
        return value - floor(value)
    }

    private static func smooth(_ value: Float) -> Float {
        value * value * (3.0 - 2.0 * value)
    }

    private static func lerp(
        _ a: Float,
        _ b: Float,
        _ t: Float
    ) -> Float {
        a + (b - a) * t
    }

    private static func valueNoise(
        _ x: Float,
        _ z: Float
    ) -> Float {

        let ix = floor(x)
        let iz = floor(z)

        let fx = smooth(x - ix)
        let fz = smooth(z - iz)

        let a = hash(ix, iz)
        let b = hash(ix + 1.0, iz)
        let c = hash(ix, iz + 1.0)
        let d = hash(ix + 1.0, iz + 1.0)

        let top = lerp(a, b, fx)
        let bottom = lerp(c, d, fx)

        return lerp(top, bottom, fz) * 2.0 - 1.0
    }

    private static func fbm(
        _ x: Float,
        _ z: Float,
        octaves: Int = 6
    ) -> Float {

        var frequency: Float = 1.0
        var amplitude: Float = 1.0

        var total: Float = 0.0
        var weight: Float = 0.0

        for _ in 0..<octaves {

            total += valueNoise(
                x * frequency,
                z * frequency
            ) * amplitude

            weight += amplitude

            frequency *= 2.0
            amplitude *= 0.5
        }

        return weight > 0.0 ? total / weight : 0.0
    }

    // MARK: - Canyon Geometry

    private static func mainCanyonCenter(
        _ z: Float
    ) -> Float {

        return
            900.0 * sin(z * 0.00045) +
            260.0 * sin(z * 0.0011 + 1.4) +
            90.0 * sin(z * 0.003)
    }

    private static func canyonCut(
        x: Float,
        z: Float,
        center: Float,
        floorWidth: Float,
        wallWidth: Float,
        depth: Float
    ) -> Float {

        let distance = abs(x - center)

        if distance >= wallWidth {
            return 0.0
        }

        if distance <= floorWidth {
            return depth
        }

        let t = (distance - floorWidth) /
                (wallWidth - floorWidth)

        let shaped = smooth(1.0 - t)

        return depth * shaped
    }

    // MARK: - Branching Ravines

    private static func branchCenter(
        _ z: Float,
        phase: Float,
        amplitude: Float,
        frequency: Float
    ) -> Float {

        return
            mainCanyonCenter(z) +
            amplitude * sin(z * frequency + phase)
    }

    private static func branchCut(
        x: Float,
        z: Float,
        phase: Float,
        amplitude: Float,
        frequency: Float,
        width: Float,
        depth: Float
    ) -> Float {

        let center = branchCenter(
            z,
            phase: phase,
            amplitude: amplitude,
            frequency: frequency
        )

        let distance = abs(x - center)

        if distance >= width {
            return 0.0
        }

        let t = distance / width
        let profile = 1.0 - smooth(t)

        return depth * profile
    }

    // MARK: - Terrain Height

    static func height(
        _ x: Float,
        _ z: Float
    ) -> Float {

        // Large geological formations.
        let continental = fbm(
            x * 0.00045,
            z * 0.00045,
            octaves: 5
        )

        // Medium-scale terrain variation.
        let regional = fbm(
            x * 0.0022 + 17.0,
            z * 0.0022 - 9.0,
            octaves: 5
        )

        // Erosion and smaller geological formations.
        let erosion = fbm(
            x * 0.009,
            z * 0.009,
            octaves: 5
        )

        // Fine surface detail.
        let detail = fbm(
            x * 0.035,
            z * 0.035,
            octaves: 4
        )

        var terrainHeight: Float =
            1050.0 +
            continental * 190.0 +
            regional * 100.0 +
            erosion * 30.0 +
            detail * 7.0

        // Main canyon.
        let mainCenter = mainCanyonCenter(z)

        let mainCut = canyonCut(
            x: x,
            z: z,
            center: mainCenter,
            floorWidth: 180.0,
            wallWidth: 1050.0,
            depth: 760.0
        )

        terrainHeight -= mainCut

        // Branching canyon systems.
        let branchOne = branchCut(
            x: x,
            z: z,
            phase: 0.8,
            amplitude: 360.0,
            frequency: 0.0008,
            width: 340.0,
            depth: 260.0
        )

        let branchTwo = branchCut(
            x: x,
            z: z,
            phase: 2.7,
            amplitude: 540.0,
            frequency: 0.0012,
            width: 270.0,
            depth: 220.0
        )

        let branchThree = branchCut(
            x: x,
            z: z,
            phase: 4.2,
            amplitude: 720.0,
            frequency: 0.00055,
            width: 230.0,
            depth: 190.0
        )

        terrainHeight -= max(
            branchOne,
            max(branchTwo, branchThree)
        )

        // Geological strata along canyon walls.
        let distanceFromCanyon = abs(x - mainCenter)

        let wallFactor = max(
            0.0,
            1.0 - distanceFromCanyon / 1050.0
        )

        let strata =
            sin(terrainHeight * 0.045) * 5.0 +
            sin(terrainHeight * 0.11) * 2.0

        terrainHeight += strata * wallFactor

        // Roughen the canyon walls.
        let wallRoughness = fbm(
            x * 0.018,
            z * 0.018,
            octaves: 4
        )

        terrainHeight += wallRoughness * 13.0 * wallFactor

        // Keep the terrain above the canyon's lower limit.
        return max(65.0, terrainHeight)
    }

    // MARK: - Terrain Coloring

    private static func terrainColor(
        height: Float,
        slope: Float,
        x: Float,
        z: Float
    ) -> SIMD4<Float> {

        let rockNoise = fbm(
            x * 0.012,
            z * 0.012,
            octaves: 3
        )

        let normalizedSlope = min(
            1.0,
            max(0.0, slope)
        )

        // Canyon floor: darker sandstone.
        let floorColor = SIMD3<Float>(
            0.38,
            0.22,
            0.12
        )

        // Canyon walls: reddish sandstone.
        let wallColor = SIMD3<Float>(
            0.62,
            0.34,
            0.19
        )

        // Upper mesas: lighter desert rock.
        let mesaColor = SIMD3<Float>(
            0.72,
            0.52,
            0.32
        )

        let floorBlend = min(
            1.0,
            max(0.0, (height - 65.0) / 300.0)
        )

        var base = SIMD3<Float>(
            lerp(floorColor.x, wallColor.x, floorBlend),
            lerp(floorColor.y, wallColor.y, floorBlend),
            lerp(floorColor.z, wallColor.z, floorBlend)
        )

        let mesaBlend = min(
            1.0,
            max(0.0, (height - 650.0) / 500.0)
        )

        base = SIMD3<Float>(
            lerp(base.x, mesaColor.x, mesaBlend),
            lerp(base.y, mesaColor.y, mesaBlend),
            lerp(base.z, mesaColor.z, mesaBlend)
        )

        // Steep slopes receive a darker rock appearance.
        let slopeDarkening = 1.0 - normalizedSlope * 0.22

        // Natural surface variation.
        let variation = 1.0 + rockNoise * 0.12

        base *= slopeDarkening * variation

        return SIMD4<Float>(
            min(1.0, max(0.0, base.x)),
            min(1.0, max(0.0, base.y)),
            min(1.0, max(0.0, base.z)),
            1.0
        )
    }

    // MARK: - Terrain Mesh

    static func mesh(
        centerX: Float,
        centerZ: Float
    ) -> SCNNode {

        let vertexCount = (cells + 1) * (cells + 1)

        var vertices: [SCNVector3] = []
        var normals: [SCNVector3] = []
        var colors: [SIMD4<Float>] = []

        vertices.reserveCapacity(vertexCount)
        normals.reserveCapacity(vertexCount)
        colors.reserveCapacity(vertexCount)

        let halfSize = Float(cells) * cell * 0.5

        let startX = centerX - halfSize
        let startZ = centerZ - halfSize

        // Generate terrain vertices.
        for row in 0...cells {

            for column in 0...cells {

                let x = startX + Float(column) * cell
                let z = startZ + Float(row) * cell

                let y = height(x, z)

                vertices.append(
                    SCNVector3(x, y, z)
                )

                // Calculate surface normal from neighboring heights.
                let sampleDistance: Float = 15.0

                let left = height(
                    x - sampleDistance,
                    z
                )

                let right = height(
                    x + sampleDistance,
                    z
                )

                let back = height(
                    x,
                    z - sampleDistance
                )

                let front = height(
                    x,
                    z + sampleDistance
                )

                let dx = (right - left) /
                         (sampleDistance * 2.0)

                let dz = (front - back) /
                         (sampleDistance * 2.0)

                let normal = simd_normalize(
                    SIMD3<Float>(
                        -dx,
                        1.0,
                        -dz
                    )
                )

                normals.append(
                    SCNVector3(
                        normal.x,
                        normal.y,
                        normal.z
                    )
                )

                let slope = sqrt(dx * dx + dz * dz)

                colors.append(
                    terrainColor(
                        height: y,
                        slope: slope,
                        x: x,
                        z: z
                    )
                )
            }
        }

        // Generate triangle indices.
        var indices: [Int32] = []
        indices.reserveCapacity(cells * cells * 6)

        for row in 0..<cells {

            for column in 0..<cells {

                let topLeft = Int32(
                    row * (cells + 1) + column
                )

                let topRight = topLeft + 1

                let bottomLeft = Int32(
                    (row + 1) * (cells + 1) + column
                )

                let bottomRight = bottomLeft + 1

                indices.append(contentsOf: [
                    topLeft,
                    bottomLeft,
                    topRight,

                    topRight,
                    bottomLeft,
                    bottomRight
                ])
            }
        }

        // Create geometry sources.
        let vertexSource = SCNGeometrySource(
            vertices: vertices
        )

        let normalSource = SCNGeometrySource(
            normals: normals
        )

        let colorData = colors.withUnsafeBufferPointer {
            Data(buffer: $0)
        }

        let colorSource = SCNGeometrySource(
            data: colorData,
            semantic: .color,
            vectorCount: colors.count,
            usesFloatComponents: true,
            componentsPerVector: 4,
            bytesPerComponent: MemoryLayout<Float>.size,
            dataOffset: 0,
            dataStride: MemoryLayout<SIMD4<Float>>.stride
        )

        let indexData = indices.withUnsafeBufferPointer {
            Data(buffer: $0)
        }

        let element = SCNGeometryElement(
            data: indexData,
            primitiveType: .triangles,
            primitiveCount: indices.count / 3,
            bytesPerIndex: MemoryLayout<Int32>.size
        )

        // Assemble the geometry.
        let geometry = SCNGeometry(
            sources: [
                vertexSource,
                normalSource,
                colorSource
            ],
            elements: [element]
        )

        let material = SCNMaterial()

        material.diffuse.contents = UIColor.white
        material.lightingModel = .lambert
        material.isDoubleSided = true
        material.writesToDepthBuffer = true
        material.readsFromDepthBuffer = true

        geometry.materials = [material]

        let terrainNode = SCNNode(
            geometry: geometry
        )

        terrainNode.name = "FractalCanyonTerrain"

        terrainNode.castsShadow = true
        terrainNode.categoryBitMask = 1

        return terrainNode
    }
}



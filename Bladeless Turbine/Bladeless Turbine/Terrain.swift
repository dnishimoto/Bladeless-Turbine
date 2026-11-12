
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
    static func canyonCenterX(z: Float) -> Float {
           mainCanyonCenter(z)
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

    // The canyon runs along world +X, matching the aircraft's forward axis.
    // This function returns the canyon's lateral center in world Z.
    private static func mainCanyonCenter(
        _ x: Float
    ) -> Float {
        return
            900.0 * sin(x * 0.00045) +
            260.0 * sin(x * 0.0011 + 1.4) +
            90.0 * sin(x * 0.003)
    }

    // MARK: - Flight Corridor and Spline Paths

    static let canyonPathHalfWidth: Float = 220.0
    static let canyonRecoveryMargin: Float = 35.0
    static let canyonVariationAmplitude: Float = 125.0

    static func canyonCenterZ(_ x: Float) -> Float {
        mainCanyonCenter(x)
    }

    static func canyonCenterPath(_ x: Float) -> SIMD3<Float> {
        SIMD3<Float>(x, 0, canyonCenterZ(x))
    }

    static func canyonLeftBoundary(_ x: Float) -> SIMD3<Float> {
        SIMD3<Float>(x, 0, canyonCenterZ(x) - canyonPathHalfWidth)
    }

    static func canyonRightBoundary(_ x: Float) -> SIMD3<Float> {
        SIMD3<Float>(x, 0, canyonCenterZ(x) + canyonPathHalfWidth)
    }

    static func canyonVariablePath(_ x: Float) -> SIMD3<Float> {
        let offset =
            canyonVariationAmplitude * sin(x * 0.0011) +
            35.0 * sin(x * 0.0027 + 1.2)
        let limit = canyonPathHalfWidth - canyonRecoveryMargin
        let safeOffset = max(-limit, min(limit, offset))
        return SIMD3<Float>(x, 0, canyonCenterZ(x) + safeOffset)
    }

    static func canyonLateralError(x: Float, z: Float) -> Float {
        z - canyonCenterZ(x)
    }

    static func canyonRecoveryStrength(x: Float, z: Float) -> Float {
        let error = abs(canyonLateralError(x: x, z: z))
        let limit = canyonPathHalfWidth - canyonRecoveryMargin
        let warning = limit * 0.45
        return min(1, max(0, (error - warning) / max(1, limit - warning)))
    }

    static func canyonFlightTarget(
        aircraftX: Float,
        aircraftZ: Float,
        lookAhead: Float
    ) -> SIMD3<Float> {
        let targetX = aircraftX + lookAhead
        let lateral = canyonLateralError(x: aircraftX, z: aircraftZ)
        let safeLimit = canyonPathHalfWidth - canyonRecoveryMargin

        // Outside the safe corridor, aim back toward the centerline.
        // Inside it, allow the flight path to vary left and right.
        let targetZ: Float
        if abs(lateral) > safeLimit {
            targetZ = canyonCenterZ(targetX)
        } else {
            targetZ = canyonVariablePath(targetX).z
        }
        return SIMD3<Float>(targetX, 0, targetZ)
    }

    private static func canyonCut(
        x: Float,
        z: Float,
        center: Float,
        floorWidth: Float,
        wallWidth: Float,
        depth: Float
    ) -> Float {

        let distance = abs(z - center)

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
        _ x: Float,
        phase: Float,
        amplitude: Float,
        frequency: Float
    ) -> Float {

        return
            mainCanyonCenter(x) +
            amplitude * sin(x * frequency + phase)
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
            x,
            phase: phase,
            amplitude: amplitude,
            frequency: frequency
        )

        let distance = abs(z - center)

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
        let mainCenter = mainCanyonCenter(x)

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
        let distanceFromCanyon = abs(z - mainCenter)

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




// MARK: - Canyon Spline Visualization

/// Draws the flight corridor over the terrain. Call update at a throttled
/// interval (about 0.2–0.5 seconds), not on every rendered frame.
enum CanyonSplineRenderer {

    private static let nodeNames = [
        "CanyonCenterSpline",
        "CanyonLeftSpline",
        "CanyonRightSpline",
        "CanyonVariableSpline",
        "CanyonRecoverySpline"
    ]

    private static func makeLine(
        name: String,
        color: UIColor,
        aircraftX: Float,
        aircraftZ: Float,
        path: (Float) -> SIMD3<Float>
    ) -> SCNNode {

        let sampleCount = 160
        let startX = aircraftX - 100
        let endX = aircraftX + 1500
        var vertices: [SCNVector3] = []
        vertices.reserveCapacity(sampleCount + 1)

        for index in 0...sampleCount {
            let t = Float(index) / Float(sampleCount)
            let x = startX + (endX - startX) * t
            let point = path(x)
            let y = Terrain.height(point.x, point.z) + 38.0
            vertices.append(SCNVector3(point.x, y, point.z))
        }

        let source = SCNGeometrySource(vertices: vertices)
        var indices: [Int32] = []
        indices.reserveCapacity(sampleCount * 2)
        for index in 0..<sampleCount {
            indices.append(Int32(index))
            indices.append(Int32(index + 1))
        }

        let element = SCNGeometryElement(
            indices: indices,
            primitiveType: .line
        )
        let geometry = SCNGeometry(sources: [source], elements: [element])
        let material = SCNMaterial()
        material.diffuse.contents = color
        material.emission.contents = color
        material.lightingModel = .constant
        material.isDoubleSided = true
        geometry.materials = [material]

        let node = SCNNode(geometry: geometry)
        node.name = name
        node.castsShadow = false
        node.categoryBitMask = 1 << 5
        return node
    }

    static func update(
        scene: SCNScene,
        aircraftPosition: SIMD3<Float>
    ) {
        for name in nodeNames {
            scene.rootNode.childNode(withName: name, recursively: false)?.removeFromParentNode()
        }

        let x = aircraftPosition.x
        let z = aircraftPosition.z
        let lateral = Terrain.canyonLateralError(x: x, z: z)
        let safeLimit = Terrain.canyonPathHalfWidth - Terrain.canyonRecoveryMargin
        let recovering = abs(lateral) > safeLimit

        let center = makeLine(
            name: "CanyonCenterSpline", color: .cyan,
            aircraftX: x, aircraftZ: z,
            path: Terrain.canyonCenterPath
        )
        let left = makeLine(
            name: "CanyonLeftSpline", color: .orange,
            aircraftX: x, aircraftZ: z,
            path: Terrain.canyonLeftBoundary
        )
        let right = makeLine(
            name: "CanyonRightSpline", color: .orange,
            aircraftX: x, aircraftZ: z,
            path: Terrain.canyonRightBoundary
        )
        let variable = makeLine(
            name: "CanyonVariableSpline", color: .yellow,
            aircraftX: x, aircraftZ: z,
            path: Terrain.canyonVariablePath
        )
        let recovery = makeLine(
            name: "CanyonRecoverySpline", color: .green,
            aircraftX: x, aircraftZ: z,
            path: { sampleX in
                let centerZ = Terrain.canyonCenterZ(sampleX)
                if recovering {
                    return SIMD3<Float>(sampleX, 0, centerZ)
                }
                let desired = Terrain.canyonVariablePath(sampleX).z
                let clamped = max(
                    centerZ - safeLimit,
                    min(centerZ + safeLimit, desired)
                )
                return SIMD3<Float>(sampleX, 0, clamped)
            }
        )

        scene.rootNode.addChildNode(center)
        scene.rootNode.addChildNode(left)
        scene.rootNode.addChildNode(right)
        scene.rootNode.addChildNode(variable)
        scene.rootNode.addChildNode(recovery)
    }
}





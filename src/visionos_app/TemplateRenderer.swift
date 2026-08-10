// TEMPORARY DIAGNOSTIC — a faithful port of Apple's "Immersive Space
// Renderer: Metal" template renderer (proven to display on this headset as
// the MetalCheck project), reduced to what display requires: the plural
// drawable query, per-drawable device anchor, rasterization rate maps,
// layered render targets with vertex amplification, per-view
// computeProjection, and real drawn geometry with reverse-Z depth.
//
// It draws a rotating colored cube on a DARK GREEN background — green so a
// working clear is distinguishable from both the engine (arena) and the old
// magenta probe. If this displays inside OUR app while the engine path does
// not, the delta is in the engine's frame structure; if even this is black,
// the delta is in the app/project configuration and no render code is at
// fault.

import ARKit
import CompositorServices
import Metal
import MetalKit
import simd

extension LayerRenderer.Clock.Instant {
    fileprivate var timeIntervalSinceEpoch: TimeInterval {
        let components = LayerRenderer.Clock.Instant.epoch.duration(to: self).components
        let nanoseconds = TimeInterval(components.attoseconds / 1_000_000_000)
        return TimeInterval(components.seconds) + (nanoseconds / TimeInterval(NSEC_PER_SEC))
    }
}

final class TemplateRenderer {
    private static let shaderSource = """
    #include <metal_stdlib>
    #include <simd/simd.h>
    using namespace metal;

    struct TemplateVertexIn {
        float3 position [[attribute(0)]];
        float2 uv       [[attribute(1)]];
    };
    struct TemplateVaryings {
        float4 position [[position]];
        float2 uv;
    };
    struct TemplateUniforms { float4x4 model; };
    struct TemplateViewProjections { float4x4 vp[2]; };

    vertex TemplateVaryings template_vertex(
        TemplateVertexIn in [[stage_in]],
        ushort amp_id [[amplification_id]],
        constant TemplateUniforms& uniforms [[buffer(2)]],
        constant TemplateViewProjections& viewProjections [[buffer(3)]])
    {
        TemplateVaryings out;
        out.position = viewProjections.vp[amp_id] * uniforms.model * float4(in.position, 1.0);
        out.uv = in.uv;
        return out;
    }

    fragment float4 template_fragment(TemplateVaryings in [[stage_in]])
    {
        return float4(in.uv.x, in.uv.y, 1.0 - in.uv.x, 1.0);
    }
    """

    static func start(_ layerRenderer: LayerRenderer) {
        Thread.detachNewThread {
            Thread.current.name = "rt.vision.template"
            let renderer = TemplateRenderer(layerRenderer)
            renderer.runLoop()
        }
    }

    private let layer: LayerRenderer
    private let device: MTLDevice
    private let queue: MTLCommandQueue
    private let pipeline: MTLRenderPipelineState
    private let depthState: MTLDepthStencilState
    private let mesh: MTKMesh
    private let arSession = ARKitSession()
    private let worldTracking = WorldTrackingProvider()
    private var rotation: Float = 0

    private init(_ layerRenderer: LayerRenderer) {
        layer = layerRenderer
        device = layerRenderer.device
        queue = device.makeCommandQueue()!

        let vertexDescriptor = MTLVertexDescriptor()
        vertexDescriptor.attributes[0].format = .float3
        vertexDescriptor.attributes[0].offset = 0
        vertexDescriptor.attributes[0].bufferIndex = 0
        vertexDescriptor.attributes[1].format = .float2
        vertexDescriptor.attributes[1].offset = 0
        vertexDescriptor.attributes[1].bufferIndex = 1
        vertexDescriptor.layouts[0].stride = 12
        vertexDescriptor.layouts[1].stride = 8

        // Compiled from source at runtime: this project has no metallib build
        // step (the engine compiles its own shaders the same way), and
        // makeDefaultLibrary() is nil without one.
        let library = try! device.makeLibrary(source: TemplateRenderer.shaderSource,
                                              options: nil)
        let pipelineDescriptor = MTLRenderPipelineDescriptor()
        pipelineDescriptor.label = "TemplatePipeline"
        pipelineDescriptor.vertexFunction = library.makeFunction(name: "template_vertex")!
        pipelineDescriptor.fragmentFunction = library.makeFunction(name: "template_fragment")!
        pipelineDescriptor.vertexDescriptor = vertexDescriptor
        pipelineDescriptor.colorAttachments[0].pixelFormat = layerRenderer.configuration.colorFormat
        pipelineDescriptor.depthAttachmentPixelFormat = layerRenderer.configuration.depthFormat
        pipelineDescriptor.maxVertexAmplificationCount = layerRenderer.properties.viewCount
        pipeline = try! device.makeRenderPipelineState(descriptor: pipelineDescriptor)

        let depthDescriptor = MTLDepthStencilDescriptor()
        depthDescriptor.depthCompareFunction = .greater
        depthDescriptor.isDepthWriteEnabled = true
        depthState = device.makeDepthStencilState(descriptor: depthDescriptor)!

        let allocator = MTKMeshBufferAllocator(device: device)
        let box = MDLMesh.newBox(withDimensions: SIMD3<Float>(0.5, 0.5, 0.5),
                                 segments: SIMD3<UInt32>(1, 1, 1),
                                 geometryType: .triangles,
                                 inwardNormals: false,
                                 allocator: allocator)
        let mdlVertexDescriptor = MTKModelIOVertexDescriptorFromMetal(vertexDescriptor)
        (mdlVertexDescriptor.attributes as! [MDLVertexAttribute])[0].name = MDLVertexAttributePosition
        (mdlVertexDescriptor.attributes as! [MDLVertexAttribute])[1].name = MDLVertexAttributeTextureCoordinate
        box.vertexDescriptor = mdlVertexDescriptor
        mesh = try! MTKMesh(mesh: box, device: device)
    }

    private func runLoop() {
        Task { try? await arSession.run([worldTracking]) }
        while true {
            switch layer.state {
            case .invalidated:
                print("[vision] TEMPLATE renderer: layer invalidated, exiting")
                return
            case .paused:
                layer.waitUntilRunning()
            default:
                autoreleasepool { renderFrame() }
            }
        }
    }

    private var frameCount: UInt64 = 0

    private func renderFrame() {
        guard let frame = layer.queryNextFrame() else { return }
        frame.startUpdate()
        rotation += 0.01
        frame.endUpdate()

        guard let timing = frame.predictTiming() else { return }
        LayerRenderer.Clock().wait(until: timing.optimalInputTime)

        guard let commandBuffer = queue.makeCommandBuffer() else { return }
        let drawables = frame.queryDrawables()
        guard !drawables.isEmpty else { return }
        frame.startSubmission()
        for drawable in drawables {
            render(drawable: drawable, commandBuffer: commandBuffer)
        }
        commandBuffer.commit()
        frame.endSubmission()

        frameCount += 1
        if frameCount % 90 == 0 {
            print("[vision] TEMPLATE renderer frame \(frameCount), drawables \(drawables.count)")
        }
    }

    private func render(drawable: LayerRenderer.Drawable, commandBuffer: MTLCommandBuffer) {
        let time = drawable.frameTiming.presentationTime.timeIntervalSinceEpoch
        drawable.deviceAnchor = worldTracking.queryDeviceAnchor(atTimestamp: time)

        var model = matrix4x4_translation(0, 0, -1.5)
            * matrix4x4_rotation(radians: rotation, axis: SIMD3<Float>(1, 1, 0))

        let anchorTransform = drawable.deviceAnchor?.originFromAnchorTransform
            ?? matrix_identity_float4x4
        var viewProjections = (matrix_identity_float4x4, matrix_identity_float4x4)
        for index in 0..<min(drawable.views.count, 2) {
            let viewMatrix = (anchorTransform * drawable.views[index].transform).inverse
            let projection = drawable.computeProjection(viewIndex: index)
            let vp = projection * viewMatrix
            if index == 0 { viewProjections.0 = vp } else { viewProjections.1 = vp }
        }

        let pass = MTLRenderPassDescriptor()
        pass.colorAttachments[0].texture = drawable.colorTextures[0]
        pass.colorAttachments[0].loadAction = .clear
        pass.colorAttachments[0].storeAction = .store
        pass.colorAttachments[0].clearColor = MTLClearColor(red: 0.0, green: 0.25, blue: 0.08, alpha: 1.0)
        pass.depthAttachment.texture = drawable.depthTextures[0]
        pass.depthAttachment.loadAction = .clear
        pass.depthAttachment.storeAction = .store
        pass.depthAttachment.clearDepth = 0.0
        pass.rasterizationRateMap = drawable.rasterizationRateMaps.first
        if layer.configuration.layout == .layered {
            pass.renderTargetArrayLength = drawable.views.count
        }

        guard let encoder = commandBuffer.makeRenderCommandEncoder(descriptor: pass) else { return }
        encoder.label = "Template Render Encoder"
        encoder.setCullMode(.back)
        encoder.setFrontFacing(.counterClockwise)
        encoder.setRenderPipelineState(pipeline)
        encoder.setDepthStencilState(depthState)

        let viewports = drawable.views.map { $0.textureMap.viewport }
        encoder.setViewports(viewports)
        if drawable.views.count > 1 {
            var mappings = (0..<drawable.views.count).map {
                MTLVertexAmplificationViewMapping(viewportArrayIndexOffset: UInt32($0),
                                                  renderTargetArrayIndexOffset: UInt32($0))
            }
            encoder.setVertexAmplificationCount(viewports.count, viewMappings: &mappings)
        }

        encoder.setVertexBytes(&model, length: MemoryLayout<matrix_float4x4>.size, index: 2)
        encoder.setVertexBytes(&viewProjections,
                               length: MemoryLayout<matrix_float4x4>.size * 2, index: 3)

        for (index, buffer) in mesh.vertexBuffers.enumerated() {
            encoder.setVertexBuffer(buffer.buffer, offset: buffer.offset, index: index)
        }
        for submesh in mesh.submeshes {
            encoder.drawIndexedPrimitives(type: submesh.primitiveType,
                                          indexCount: submesh.indexCount,
                                          indexType: submesh.indexType,
                                          indexBuffer: submesh.indexBuffer.buffer,
                                          indexBufferOffset: submesh.indexBuffer.offset)
        }
        encoder.endEncoding()
        drawable.encodePresent(commandBuffer: commandBuffer)
    }
}

private func matrix4x4_rotation(radians: Float, axis: SIMD3<Float>) -> matrix_float4x4 {
    let unitAxis = normalize(axis)
    let ct = cosf(radians)
    let st = sinf(radians)
    let ci = 1 - ct
    let x = unitAxis.x, y = unitAxis.y, z = unitAxis.z
    return .init(columns: (vector_float4(ct + x * x * ci, y * x * ci + z * st, z * x * ci - y * st, 0),
                           vector_float4(x * y * ci - z * st, ct + y * y * ci, z * y * ci + x * st, 0),
                           vector_float4(x * z * ci + y * st, y * z * ci - x * st, ct + z * z * ci, 0),
                           vector_float4(0, 0, 0, 1)))
}

private func matrix4x4_translation(_ x: Float, _ y: Float, _ z: Float) -> matrix_float4x4 {
    return .init(columns: (vector_float4(1, 0, 0, 0),
                           vector_float4(0, 1, 0, 0),
                           vector_float4(0, 0, 1, 0),
                           vector_float4(x, y, z, 1)))
}

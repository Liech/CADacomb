import SVORaycast.LOD.PixelFootprintLODPolicy;
import SVORaycast.Pipeline.RaycastFragmentOutput;
import SVORaycast.Pipeline.SVORaycastParams;
import SVORaycast.Pipeline.SceneUniform;
import SVORaycast.Shading.DirectionalSVOShading;
import SVORaycast.Shading.SVOShadeContext;
import SVORaycast.Storage.FlatSVOStorage;
import SVORaycast.Storage.GPUSVONode;
import SVORaycast.TetraProxy.GPUTetraVertex;
import SVORaycast.TetraProxy.GPUTetrahedron;
import SVORaycast.TetraProxy.TetraProxyVertexOutput;
import SVORaycast.TetraProxy.TetraRaycastResult;
import SVORaycast.TetraProxy.TetrahedronProxy;

[[vk::binding(0, 0)]] ConstantBuffer<SceneUniform>     scene;
[[vk::binding(1, 0)]] ConstantBuffer<SVORaycastParams> raycastParams;
[[vk::binding(2, 0)]] StructuredBuffer<GPUSVONode>     svoNodes;
[[vk::binding(3, 0)]] StructuredBuffer<GPUTetraVertex> tetraVertices;
[[vk::binding(4, 0)]] StructuredBuffer<GPUTetrahedron> tetrahedra;

[shader("fragment")]
RaycastFragmentOutput main(TetraProxyVertexOutput input)
{
    float2 ndc = (input.position.xy / max(scene.resolution, float2(1.0))) * 2.0 - 1.0;

    float4 farView = mul(scene.invProjectionMatrix, float4(ndc, 1.0, 1.0));
    farView /= farView.w;

    float3 worldOrigin = mul(scene.invViewMatrix, float4(0.0, 0.0, 0.0, 1.0)).xyz;
    float3 worldFar    = mul(scene.invViewMatrix, float4(farView.xyz, 1.0)).xyz;
    float3 worldDir    = normalize(worldFar - worldOrigin);

    FlatSVOStorage storage;
    storage.dimension = raycastParams.svoDimension;
    storage.nodes     = svoNodes;

    float cotFovHalf = max(abs(scene.projectionMatrix[1][1]), 1e-4);
    float projFactor = 0.5 * cotFovHalf * max(scene.resolution.y, 1.0);

    PixelFootprintLODPolicy lodPolicy;
    lodPolicy.pixelConeSlope = raycastParams.pixelLodScale / max(projFactor, 1.0);
    lodPolicy.maxDepth       = raycastParams.maxDepth;

    TetrahedronProxy proxy;
    proxy.modelMatrix    = raycastParams.modelMatrix;
    proxy.invModelMatrix = raycastParams.invModelMatrix;
    proxy.normalMatrix   = raycastParams.normalMatrix;
    proxy.tetraVertices  = tetraVertices;
    proxy.tetrahedra     = tetrahedra;

    TetraRaycastResult traceResult = proxy.trace(worldOrigin,
                                                 worldDir,
                                                 input.tetAndFace,
                                                 raycastParams.cameraTetIndex,
                                                 storage,
                                                 lodPolicy,
                                                 raycastParams.maxSteps,
                                                 64u);
    if (!traceResult.hit)
    {
        discard;
    }

    float3 hitNormalView = normalize(mul((float3x3)scene.viewMatrix, traceResult.hitNormalWorld));

    SVOShadeContext shadeCtx;
    shadeCtx.hit        = traceResult.svoHit;
    shadeCtx.posWorld   = traceResult.hitWorldPos;
    shadeCtx.normalView = hitNormalView;
    shadeCtx.baseColor  = raycastParams.baseColor.rgb;

    DirectionalSVOShading shading;
    float3                shadedColor = shading.shade(shadeCtx);

    float4 clipPos = mul(scene.projectionMatrix, mul(scene.viewMatrix, float4(traceResult.hitWorldPos, 1.0)));

    RaycastFragmentOutput output;
    output.color  = float4(shadedColor, 1.0);
    output.normal = float4(hitNormalView * 0.5 + 0.5, 1.0);
    output.depth  = clipPos.z / clipPos.w;
    return output;
}

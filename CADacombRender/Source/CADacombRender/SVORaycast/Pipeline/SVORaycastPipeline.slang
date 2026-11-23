module SVORaycastPipeline;

import SVORaycast.Core.RayHit;
import SVORaycast.LOD.ILODPolicy;
import SVORaycast.Pipeline.ProxyVertexInput;
import SVORaycast.Pipeline.ProxyVertexOutput;
import SVORaycast.Pipeline.RaycastFragmentOutput;
import SVORaycast.Pipeline.SVORaycastParams;
import SVORaycast.Pipeline.SceneUniform;
import SVORaycast.Proxy.IRayProxy;
import SVORaycast.Proxy.ProxyRaySetup;
import SVORaycast.Shading.ISVOShading;
import SVORaycast.Shading.SVOShadeContext;
import SVORaycast.Storage.GPUSVONode;
import SVORaycast.Storage.ISVOStorage;
import SVORaycast.Traversal.SVODDATraversal;

[[vk::binding(0, 0)]] public ConstantBuffer<SceneUniform>     scene;
[[vk::binding(1, 0)]] public ConstantBuffer<SVORaycastParams> raycastParams;
[[vk::binding(2, 0)]] public StructuredBuffer<GPUSVONode>     svoNodes;

public ProxyVertexOutput buildProxyVertex(ProxyVertexInput input)
{
    ProxyVertexOutput output;
    float3 localPos = raycastParams.aabbMin.xyz + input.unitPosition * raycastParams.aabbSize.xyz;
    float4 worldPos = mul(raycastParams.modelMatrix, float4(localPos, 1.0));
    float4 viewPos  = mul(scene.viewMatrix, worldPos);
    output.position = mul(scene.projectionMatrix, viewPos);
    return output;
}

public RaycastFragmentOutput executeRaycastFragment<TStorage : ISVOStorage, TProxy : IRayProxy, TLOD : ILODPolicy, TShading : ISVOShading>(
    ProxyVertexOutput input,
    TStorage          storage,
    TProxy            proxy,
    TLOD              lodPolicy,
    TShading          shading)
{
    float2 ndc = (input.position.xy / max(scene.resolution, float2(1.0))) * 2.0 - 1.0;

    float4 nearView = mul(scene.invProjectionMatrix, float4(ndc, 0.0, 1.0));
    nearView /= nearView.w;
    float4 farView = mul(scene.invProjectionMatrix, float4(ndc, 1.0, 1.0));
    farView /= farView.w;

    float3 worldOrigin = mul(scene.invViewMatrix, float4(nearView.xyz, 1.0)).xyz;
    float3 worldFar    = mul(scene.invViewMatrix, float4(farView.xyz, 1.0)).xyz;
    float3 worldDir    = normalize(worldFar - worldOrigin);

    ProxyRaySetup raySetup = proxy.buildVoxelRay(worldOrigin, worldDir);
    if (!raySetup.valid)
    {
        discard;
    }

    RayHit hit = traverseSVO(raySetup.voxelRay, storage, lodPolicy, raycastParams.maxSteps);
    if (!hit.hit)
    {
        discard;
    }

    float3 hitWorldPos   = proxy.voxelPosToWorld(hit.positionLocal);
    float3 hitNormalWrld = proxy.voxelNormalToWorld(hit.normalLocal);
    float3 hitNormalView = normalize(mul((float3x3)scene.viewMatrix, hitNormalWrld));

    SVOShadeContext shadeCtx;
    shadeCtx.hit        = hit;
    shadeCtx.posWorld   = hitWorldPos;
    shadeCtx.normalView = hitNormalView;
    shadeCtx.baseColor  = raycastParams.baseColor.rgb;

    float3 shadedColor = shading.shade(shadeCtx);

    float4 clipPos = mul(scene.projectionMatrix, mul(scene.viewMatrix, float4(hitWorldPos, 1.0)));

    RaycastFragmentOutput output;
    output.color  = float4(shadedColor, 1.0);
    output.normal = float4(hitNormalView * 0.5 + 0.5, 1.0);
    output.depth  = clipPos.z / clipPos.w;
    return output;
}

import SVORaycast.Core.RayHit;
import SVORaycast.LOD.PixelFootprintLODPolicy;
import SVORaycast.Pipeline.SVORaycastParams;
import SVORaycast.Pipeline.SceneUniform;
import SVORaycast.Proxy.AABBProxy;
import SVORaycast.Proxy.ProxyRaySetup;
import SVORaycast.Storage.FlatSVOStorage;
import SVORaycast.Storage.GPUSVONode;
import SVORaycast.Traversal.SVODDATraversal;

[[vk::binding(0, 0)]] ConstantBuffer<SceneUniform>     scene;
[[vk::binding(1, 0)]] ConstantBuffer<SVORaycastParams> raycastParams;
[[vk::binding(2, 0)]] StructuredBuffer<GPUSVONode>     svoNodes;
[[vk::binding(3, 0)]] RWStructuredBuffer<float>        beamDepths;

[shader("compute")]
[numthreads(8, 8, 1)]
void main(uint3 dispatchThreadId : SV_DispatchThreadID)
{
    uint bx = dispatchThreadId.x;
    uint by = dispatchThreadId.y;
    if (bx >= raycastParams.beamWidth || by >= raycastParams.beamHeight)
    {
        return;
    }

    uint outIdx = by * raycastParams.beamWidth + bx;

    float2 res      = max(scene.resolution, float2(1.0));
    float2 pixelPos = min(float2(bx * raycastParams.beamTileSize, by * raycastParams.beamTileSize), res);
    float2 ndc      = (pixelPos / res) * 2.0 - 1.0;

    float4 nearView = mul(scene.invProjectionMatrix, float4(ndc, 0.0, 1.0));
    nearView /= nearView.w;
    float4 farView = mul(scene.invProjectionMatrix, float4(ndc, 1.0, 1.0));
    farView /= farView.w;

    float3 worldOrigin = mul(scene.invViewMatrix, float4(nearView.xyz, 1.0)).xyz;
    float3 worldFar    = mul(scene.invViewMatrix, float4(farView.xyz, 1.0)).xyz;
    float3 worldDir    = normalize(worldFar - worldOrigin);

    AABBProxy proxy;
    proxy.modelMatrix    = raycastParams.modelMatrix;
    proxy.invModelMatrix = raycastParams.invModelMatrix;
    proxy.normalMatrix   = raycastParams.normalMatrix;
    proxy.aabbMin        = raycastParams.aabbMin.xyz;
    proxy.aabbSize       = raycastParams.aabbSize.xyz;
    proxy.svoDimension   = float(raycastParams.svoDimension);

    ProxyRaySetup raySetup = proxy.buildVoxelRay(worldOrigin, worldDir);
    if (!raySetup.valid)
    {
        beamDepths[outIdx] = 0.0;
        return;
    }

    FlatSVOStorage storage;
    storage.dimension = raycastParams.svoDimension;
    storage.nodes     = svoNodes;

    float cotFovHalf     = max(abs(scene.projectionMatrix[1][1]), 1e-4);
    float projFactor     = 0.5 * cotFovHalf * max(scene.resolution.y, 1.0);
    float beamRadiusPx   = float(raycastParams.beamTileSize) * 1.5;

    PixelFootprintLODPolicy lodPolicy;
    lodPolicy.pixelConeSlope = max(beamRadiusPx, raycastParams.pixelLodScale) / max(projFactor, 1.0);
    lodPolicy.maxDepth       = raycastParams.maxDepth;

    RayHit hit = traverseSVO(raySetup.voxelRay, storage, lodPolicy, raycastParams.maxSteps);
    if (!hit.hit)
    {
        beamDepths[outIdx] = 1e30;
        return;
    }

    float hitNodeSize  = float(raycastParams.svoDimension) / exp2(float(hit.depth));
    beamDepths[outIdx] = max(0.0, hit.t - hitNodeSize * 2.0);
}

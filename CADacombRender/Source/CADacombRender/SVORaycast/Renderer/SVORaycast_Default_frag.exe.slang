import SVORaycast.Beam.CoarseBeamAccelerator;
import SVORaycast.LOD.PixelFootprintLODPolicy;
import SVORaycast.Pipeline.ProxyVertexOutput;
import SVORaycast.Pipeline.RaycastFragmentOutput;
import SVORaycast.Pipeline.SVORaycastPipeline;
import SVORaycast.Proxy.AABBProxy;
import SVORaycast.Shading.DirectionalSVOShading;
import SVORaycast.Storage.FlatSVOStorage;

[shader("fragment")]
RaycastFragmentOutput main(ProxyVertexOutput input)
{
    FlatSVOStorage storage;
    storage.dimension = raycastParams.svoDimension;
    storage.nodes     = svoNodes;

    AABBProxy proxy;
    proxy.modelMatrix    = raycastParams.modelMatrix;
    proxy.invModelMatrix = raycastParams.invModelMatrix;
    proxy.normalMatrix   = raycastParams.normalMatrix;
    proxy.aabbMin        = raycastParams.aabbMin.xyz;
    proxy.aabbSize       = raycastParams.aabbSize.xyz;
    proxy.svoDimension   = float(raycastParams.svoDimension);

    float cotFovHalf = max(abs(scene.projectionMatrix[1][1]), 1e-4);
    float projFactor = 0.5 * cotFovHalf * max(scene.resolution.y, 1.0);

    PixelFootprintLODPolicy lodPolicy;
    lodPolicy.pixelConeSlope = raycastParams.pixelLodScale / max(projFactor, 1.0);
    lodPolicy.maxDepth       = raycastParams.maxDepth;

    CoarseBeamAccelerator beamAccelerator;
    beamAccelerator.beamWidth    = raycastParams.beamWidth;
    beamAccelerator.beamHeight   = raycastParams.beamHeight;
    beamAccelerator.beamTileSize = raycastParams.beamTileSize;
    beamAccelerator.beamEnabled  = raycastParams.beamEnabled;
    beamAccelerator.beamDepths   = beamDepths;

    DirectionalSVOShading shading;

    return executeRaycastFragment(input, storage, proxy, lodPolicy, shading, beamAccelerator);
}

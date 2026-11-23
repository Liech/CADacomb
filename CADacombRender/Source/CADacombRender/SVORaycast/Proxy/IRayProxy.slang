module IRayProxy;

import SVORaycast.Proxy.ProxyRaySetup;

// Abstract proxy geometry interface mapping world rays to SVO space.
public interface IRayProxy
{
    ProxyRaySetup buildVoxelRay(float3 worldOrigin, float3 worldDirection);
    float3        voxelPosToWorld(float3 voxelPos);
    float3        voxelNormalToWorld(float3 voxelNormal);
};

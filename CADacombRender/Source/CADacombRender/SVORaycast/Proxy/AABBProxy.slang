module AABBProxy;

import SVORaycast.Core.Ray;
import SVORaycast.Proxy.IRayProxy;
import SVORaycast.Proxy.ProxyRaySetup;

// Rigid axis-aligned bounding box proxy mapping world rays into [0, dimension]^3 SVO voxel space.
public struct AABBProxy : IRayProxy
{
    public ProxyRaySetup buildVoxelRay(float3 worldOrigin, float3 worldDirection)
    {
        ProxyRaySetup setup;
        setup.valid             = false;
        setup.worldToVoxelScale = 1.0;

        float3 localOrigin = mul(invModelMatrix, float4(worldOrigin, 1.0)).xyz;
        float3 localDir    = mul((float3x3)invModelMatrix, worldDirection);

        float3 invSize     = float3(svoDimension) / max(aabbSize, float3(1e-6));
        float3 voxelOrigin = (localOrigin - aabbMin) * invSize;
        float3 voxelDirRaw = localDir * invSize;
        float  voxelDirLen = length(voxelDirRaw);

        if (voxelDirLen < 1e-8)
        {
            setup.voxelRay = Ray::create(voxelOrigin, float3(0.0, 0.0, 1.0), 0.0, 0.0);
            return setup;
        }

        float3 voxelDir = voxelDirRaw / voxelDirLen;
        Ray    ray      = Ray::create(voxelOrigin, voxelDir, 0.0, 1e30);

        float3 t0    = (float3(0.0) - ray.origin) * ray.invDirection;
        float3 t1    = (float3(svoDimension) - ray.origin) * ray.invDirection;
        float3 tLow  = min(t0, t1);
        float3 tHigh = max(t0, t1);

        float tEnter = max(max(tLow.x, tLow.y), max(tLow.z, 0.0));
        float tExit  = min(min(tHigh.x, tHigh.y), tHigh.z);

        if (tExit <= tEnter)
        {
            setup.voxelRay = ray;
            return setup;
        }

        ray.tMin                = tEnter;
        ray.tMax                = tExit;
        setup.valid             = true;
        setup.voxelRay          = ray;
        setup.worldToVoxelScale = voxelDirLen;
        return setup;
    }

    public float3 voxelPosToWorld(float3 voxelPos)
    {
        float3 localPos = aabbMin + (voxelPos / max(svoDimension, 1.0)) * aabbSize;
        return mul(modelMatrix, float4(localPos, 1.0)).xyz;
    }

    public float3 voxelNormalToWorld(float3 voxelNormal)
    {
        float3 scaledNormal = voxelNormal / max(aabbSize, float3(1e-6));
        return normalize(mul((float3x3)normalMatrix, scaledNormal));
    }

    public float4x4 modelMatrix;
    public float4x4 invModelMatrix;
    public float4x4 normalMatrix;
    public float3   aabbMin;
    public float3   aabbSize;
    public float    svoDimension;
};

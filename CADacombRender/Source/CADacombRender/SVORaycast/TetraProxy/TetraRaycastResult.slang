module TetraRaycastResult;

import SVORaycast.Core.RayHit;

// Result of piecewise-linear ray traversal through a deformed tetrahedron cage.
public struct TetraRaycastResult
{
    public bool   hit;
    public RayHit svoHit;
    public float3 hitWorldPos;
    public float3 hitNormalWorld;
};

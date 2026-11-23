module ProxyRaySetup;

import SVORaycast.Core.Ray;

// Result of mapping a world-space camera ray through a proxy volume into SVO voxel space.
public struct ProxyRaySetup
{
    public bool  valid;
    public Ray   voxelRay;
    public float worldToVoxelScale;
};

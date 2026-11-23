module PixelFootprintLODPolicy;

import SVORaycast.LOD.ILODPolicy;

// Continuous screen-space pixel-footprint LOD policy.
public struct PixelFootprintLODPolicy : ILODPolicy
{
    public bool shouldStopAtNode(float tVoxel, float nodeSizeVoxels, uint depth)
    {
        if (maxDepth > 0u && depth >= maxDepth)
        {
            return true;
        }
        return nodeSizeVoxels <= (tVoxel * pixelConeSlope);
    }

    public float pixelConeSlope;
    public uint  maxDepth;
};

module SVOShadeContext;

import SVORaycast.Core.RayHit;

// Shading inputs for an SVO ray hit.
public struct SVOShadeContext
{
    public RayHit hit;
    public float3 posWorld;
    public float3 normalView;
    public float3 baseColor;
};

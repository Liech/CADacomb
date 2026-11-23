module ISVOShading;

import SVORaycast.Shading.SVOShadeContext;

// Abstract shading interface for SVO raycast hits.
public interface ISVOShading
{
    float3 shade(SVOShadeContext ctx);
};

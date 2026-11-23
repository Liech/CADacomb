module DirectionalSVOShading;

import SVORaycast.Shading.ISVOShading;
import SVORaycast.Shading.SVOShadeContext;

// Directional and ambient lighting matching CADaver default view-space lighting.
public struct DirectionalSVOShading : ISVOShading
{
    public float3 shade(SVOShadeContext ctx)
    {
        float3 lightDirView = normalize(float3(0.3, 0.8, 0.5));
        float  diff         = max(dot(ctx.normalView, lightDirView), 0.0);
        float  ambient      = 0.25;
        float  lighting     = ambient + 0.75 * diff;
        return ctx.baseColor * lighting;
    }
};

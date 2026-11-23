module CoarseBeamAccelerator;

import SVORaycast.Beam.IBeamAccelerator;

// Conservative 4-corner coarse beam depth lookup skipping empty space before SVO traversal.
public struct CoarseBeamAccelerator : IBeamAccelerator
{
    public float queryStartT(float2 fragCoord, float defaultTMin)
    {
        if (beamEnabled == 0u || beamWidth < 2u || beamHeight < 2u)
        {
            return defaultTMin;
        }

        uint2 tile = uint2(fragCoord) / beamTileSize;
        uint  x0   = min(tile.x, beamWidth - 2u);
        uint  y0   = min(tile.y, beamHeight - 2u);
        uint  x1   = x0 + 1u;
        uint  y1   = y0 + 1u;

        float d00 = beamDepths[y0 * beamWidth + x0];
        float d10 = beamDepths[y0 * beamWidth + x1];
        float d01 = beamDepths[y1 * beamWidth + x0];
        float d11 = beamDepths[y1 * beamWidth + x1];

        return min(min(d00, d10), min(d01, d11));
    }

    public bool isTileEmpty(float startT)
    {
        return startT >= 1e29;
    }

    public uint                    beamWidth;
    public uint                    beamHeight;
    public uint                    beamTileSize;
    public uint                    beamEnabled;
    public StructuredBuffer<float> beamDepths;
};

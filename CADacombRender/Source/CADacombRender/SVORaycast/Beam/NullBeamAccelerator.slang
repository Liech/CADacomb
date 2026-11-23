module NullBeamAccelerator;

import SVORaycast.Beam.IBeamAccelerator;

// No-op beam accelerator starting every ray directly at the proxy entry distance.
public struct NullBeamAccelerator : IBeamAccelerator
{
    public float queryStartT(float2 fragCoord, float defaultTMin)
    {
        return defaultTMin;
    }

    public bool isTileEmpty(float startT)
    {
        return false;
    }
};

module IBeamAccelerator;

// Abstract ray start-distance acceleration interface (e.g. coarse beam depth prepass).
public interface IBeamAccelerator
{
    float queryStartT(float2 fragCoord, float defaultTMin);
    bool  isTileEmpty(float startT);
};

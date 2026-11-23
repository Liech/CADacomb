module RayHit;

// Intersection hit record returned by SVO traversal.
public struct RayHit
{
    public static RayHit miss(uint stepCount)
    {
        RayHit result;
        result.hit           = false;
        result.t             = 0.0;
        result.positionLocal = float3(0.0, 0.0, 0.0);
        result.normalLocal   = float3(0.0, 1.0, 0.0);
        result.depth         = 0u;
        result.steps         = stepCount;
        result.value         = 0u;
        return result;
    }

    public bool   hit;
    public float  t;
    public float3 positionLocal;
    public float3 normalLocal;
    public uint   depth;
    public uint   steps;
    public uint   value;
};

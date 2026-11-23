module SVORaycastParams;

// GPU uniform layout matching C++ SVORaycastParams.
public struct SVORaycastParams
{
    public float4x4 modelMatrix;
    public float4x4 invModelMatrix;
    public float4x4 normalMatrix;
    public float4   aabbMin;
    public float4   aabbSize;
    public float4   baseColor;
    public uint     svoDimension;
    public uint     maxSteps;
    public uint     maxDepth;
    public float    pixelLodScale;
};

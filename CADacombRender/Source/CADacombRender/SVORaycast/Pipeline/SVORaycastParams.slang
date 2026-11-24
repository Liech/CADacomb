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
    public uint     beamWidth;
    public uint     beamHeight;
    public uint     beamTileSize;
    public uint     beamEnabled;
    public int      cameraTetIndex;
    public int      padding0;
    public int      padding1;
    public int      padding2;
};

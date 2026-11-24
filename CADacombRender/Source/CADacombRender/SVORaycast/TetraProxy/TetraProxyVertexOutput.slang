module TetraProxyVertexOutput;

// Vertex-to-fragment payload carrying clip position and entry tetrahedron index.
public struct TetraProxyVertexOutput
{
    public float4               position   : SV_Position;
    public nointerpolation uint tetAndFace : LOCATION0;
};

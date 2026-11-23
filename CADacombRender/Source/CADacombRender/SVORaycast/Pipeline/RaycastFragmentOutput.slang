module RaycastFragmentOutput;

// Fragment shader output writing color, view-space normal, and exact ray-hit depth.
public struct RaycastFragmentOutput
{
    public float4 color  : SV_Target0;
    public float4 normal : SV_Target1;
    public float  depth  : SV_Depth;
};

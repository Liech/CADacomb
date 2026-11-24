module GPUTetraVertex;

// Packed 32-byte GPU tetrahedron cage vertex matching C++ GPUTetraVertex.
public struct GPUTetraVertex
{
    public float4 restVoxelPos;
    public float4 deformedLocalPos;
};

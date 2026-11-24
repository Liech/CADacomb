module GPUTetrahedron;

// Packed 32-byte GPU tetrahedron topology matching C++ GPUTetrahedron.
public struct GPUTetrahedron
{
    public uint4 vertexIndices;
    public int4  neighborTets;
};

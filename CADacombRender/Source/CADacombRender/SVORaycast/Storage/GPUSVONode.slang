module GPUSVONode;

// Packed 8-byte GPU SVO node layout matching C++ GPUSVONode (24-bit materialId + 8-bit child validMask).
public struct GPUSVONode
{
    public uint getMaterialId()
    {
        return payload & 0x00FFFFFFu;
    }

    public uint getValidMask()
    {
        return payload >> 24u;
    }

    public bool isLeaf()
    {
        return childPointer == 0xFFFFFFFFu;
    }

    public uint childPointer;
    public uint payload;
};

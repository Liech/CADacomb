module GPUSVONode;

// Packed 8-byte GPU SVO node layout matching C++ GPUSVONode.
public struct GPUSVONode
{
    public uint childPointer;
    public uint value;
};

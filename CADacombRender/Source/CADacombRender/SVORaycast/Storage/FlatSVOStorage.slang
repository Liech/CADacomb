module FlatSVOStorage;

import SVORaycast.Storage.GPUSVONode;
import SVORaycast.Storage.ISVOStorage;

// Flat StructuredBuffer implementation of ISVOStorage for VRAM-resident SVOs.
public struct FlatSVOStorage : ISVOStorage
{
    public uint getDimension()
    {
        return dimension;
    }

    public GPUSVONode getNode(uint index)
    {
        return nodes[index];
    }

    public bool isSolid(uint value)
    {
        return value != 0u;
    }

    public uint                         dimension;
    public StructuredBuffer<GPUSVONode> nodes;
};

module ISVOStorage;

import SVORaycast.Storage.GPUSVONode;

// Abstract SVO node storage interface enabling flat VRAM buffers or paged GPU caches.
public interface ISVOStorage
{
    uint       getDimension();
    GPUSVONode getNode(uint index);
    bool       isSolid(uint value);
};

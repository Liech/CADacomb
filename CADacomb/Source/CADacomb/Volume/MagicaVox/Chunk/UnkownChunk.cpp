#include "UnkownChunk.h"

#include "CADacomb/Volume/MagicaVox/IO/Reader.h"

namespace MagicaVoxImporter
{
    UnkownChunk::~UnkownChunk() {}

    std::string UnkownChunk::getID() const
    {
        return "????";
    }

    void UnkownChunk::read(Reader own, Reader child)
    {
        Chunk::read(own, child);
    }
}
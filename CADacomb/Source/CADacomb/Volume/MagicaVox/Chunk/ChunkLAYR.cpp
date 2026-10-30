#include "ChunkLAYR.h"

#include "CADacomb/Volume/MagicaVox/IO/Reader.h"

namespace MagicaVoxImporter
{
    ChunkLAYR::~ChunkLAYR() {}

    std::string ChunkLAYR::getID() const
    {
        return "LAYR";
    }

    void ChunkLAYR::read(Reader own, Reader child)
    {
        Chunk::read(own, child);

        layerId    = own.readInt();
        attributes = own.readDict();
        own.readInt();
    }
}
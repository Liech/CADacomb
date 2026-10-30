#include "ChunkPACK.h"
#include "CADacomb/Volume/MagicaVox/IO/Reader.h"

namespace MagicaVoxImporter
{
    ChunkPACK::~ChunkPACK() {}

    std::string ChunkPACK::getID() const
    {
        return "PACK";
    }

    void ChunkPACK::read(Reader own, Reader child)
    {
        Chunk::read(own, child);
        numModels = own.readInt();
    }
}
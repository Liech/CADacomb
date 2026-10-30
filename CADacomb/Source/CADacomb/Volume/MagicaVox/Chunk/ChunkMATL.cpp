#include "ChunkMATL.h"

#include "CADacomb/Volume/MagicaVox/IO/Reader.h"
#include <map>

namespace MagicaVoxImporter
{
    ChunkMATL::~ChunkMATL() {}

    std::string ChunkMATL::getID() const
    {
        return "MATL";
    }

    void ChunkMATL::read(Reader own, Reader child)
    {
        Chunk::read(own, child);

        materialId = own.readInt();
        properties = own.readDict();
    }
}
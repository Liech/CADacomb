#include "ChunkrOBJ.h"

#include "CADacomb/Volume/MagicaVox/IO/Reader.h"

namespace MagicaVoxImporter
{
    ChunkrOBJ::~ChunkrOBJ() {}

    std::string ChunkrOBJ::getID() const
    {
        return "rOBJ";
    }

    void ChunkrOBJ::read(Reader own, Reader child)
    {
        Chunk::read(own, child);
    }
}
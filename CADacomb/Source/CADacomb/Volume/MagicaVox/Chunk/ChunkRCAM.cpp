#include "ChunkRCAM.h"
#include "CADacomb/Volume/MagicaVox/IO/Reader.h"

namespace MagicaVoxImporter
{
    ChunkRCAM::~ChunkRCAM() {}

    std::string ChunkRCAM::getID() const
    {
        return "RCAM";
    }

    void ChunkRCAM::read(Reader own, Reader child)
    {
        Chunk::read(own, child);

        cameraId   = own.readInt();
        attributes = own.readDict();
    }
}
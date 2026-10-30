#include "ChunkNOTE.h"
#include "CADacomb/Volume/MagicaVox/IO/Reader.h"

namespace MagicaVoxImporter
{
    ChunkNOTE::~ChunkNOTE() {}

    std::string ChunkNOTE::getID() const
    {
        return "NOTE";
    }

    void ChunkNOTE::read(Reader own, Reader child)
    {
        Chunk::read(own, child);

        int amountColors = own.readInt();

        colorNames.resize(amountColors);
        for (int i = 0; i < amountColors; i++)
            colorNames[i] = own.readString();
    }
}
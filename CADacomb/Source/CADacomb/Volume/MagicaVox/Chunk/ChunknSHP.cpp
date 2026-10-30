#include "ChunknSHP.h"

#include "CADacomb/Volume/MagicaVox/IO/Reader.h"

namespace MagicaVoxImporter
{
    ChunknSHP::~ChunknSHP() {}

    std::string ChunknSHP::getID() const
    {
        return "nSHP";
    }

    void ChunknSHP::read(Reader own, Reader child)
    {
        Chunk::read(own, child);
        id                 = own.readInt();
        attributes         = own.readDict();
        int numberOfModels = own.readInt();
        models.resize(numberOfModels);
        for (int i = 0; i < numberOfModels; i++)
        {
            int  modelId         = own.readInt();
            auto modelAttributes = own.readDict();
            models[i]            = std::make_pair(modelId, modelAttributes);
        }
    }
}
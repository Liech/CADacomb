#pragma once

#include "Chunk.h"
#include <map>

namespace MagicaVoxImporter
{
    // Material Chunk
    class ChunkMATL : public Chunk
    {
      public:
        virtual ~ChunkMATL();

        std::string getID() const override;
        void        read(Reader own, Reader child) override;

        int                                materialId;
        std::map<std::string, std::string> properties;
    };
}
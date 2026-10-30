#pragma once
#include "Chunk.h"
#include <map>

namespace MagicaVoxImporter
{
    // Layer Chunk
    class ChunkLAYR : public Chunk
    {
      public:
        virtual ~ChunkLAYR();

        std::string getID() const override;
        void        read(Reader own, Reader child) override;

        int                                layerId;
        std::map<std::string, std::string> attributes;
    };
}
#pragma once

#include "Chunk.h"

namespace MagicaVoxImporter
{
    // main can do nothing. only having children
    class ChunkMAIN : public Chunk
    {
      public:
        virtual ~ChunkMAIN();

        std::string getID() const override;
        void        write(std::vector<unsigned char>&) const override;
    };
}
#pragma once

#include "Chunk.h"

namespace MagicaVoxImporter
{
    // Render Objects Chunk
    class ChunkrOBJ : public Chunk
    {
      public:
        virtual ~ChunkrOBJ();

        std::string getID() const override;
        void        read(Reader own, Reader child) override;
    };
}
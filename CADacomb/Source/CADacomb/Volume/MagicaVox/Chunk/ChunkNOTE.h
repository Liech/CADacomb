#pragma once

#include <map>
#include <string>

#include "Chunk.h"

namespace MagicaVoxImporter
{
    // Palette Note Chunk (defines palette color names)
    class ChunkNOTE : public Chunk
    {
      public:
        virtual ~ChunkNOTE();

        std::string getID() const override;
        void        read(Reader own, Reader child) override;

        std::vector<std::string> colorNames;
    };
}
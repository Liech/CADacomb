#pragma once

#include "Chunk.h"

namespace MagicaVoxImporter
{
    class UnkownChunk : public Chunk
    {
      public:
        virtual ~UnkownChunk();

        std::string getID() const override;
        void        read(Reader own, Reader child) override;
    };
}
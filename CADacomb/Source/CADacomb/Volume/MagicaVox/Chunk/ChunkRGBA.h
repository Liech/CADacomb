#pragma once

#include "CADacomb/Volume/MagicaVox/Chunk/Color.h"
#include "Chunk.h"

namespace MagicaVoxImporter
{
    // Chunk id 'RGBA' : palette. Default palette is used if chunk 'RGBA' is absent
    class ChunkRGBA : public Chunk
    {
      public:
        virtual ~ChunkRGBA();

        std::string getID() const override;
        void        read(Reader own, Reader child) override;

        static std::unique_ptr<ChunkRGBA> getDefault();

      private:
        static Color intToColor(unsigned int value);

      public:
        Color palette[256];

      private:
        static const unsigned int defaultPalette[256];
    };
}
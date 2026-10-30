#pragma once

#include <map>
#include <string>

#include "Chunk.h"

namespace MagicaVoxImporter
{
    // Render Camera Chunk
    class ChunkRCAM : public Chunk
    {
      public:
        virtual ~ChunkRCAM();

        std::string getID() const override;
        void        read(Reader own, Reader child) override;

        int                                cameraId;
        std::map<std::string, std::string> attributes;
    };
}
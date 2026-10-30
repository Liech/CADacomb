#pragma once

#include <memory>
#include <string>

namespace MagicaVoxImporter
{
    class Chunk;

    class ChunkFactory
    {
      public:
        virtual ~ChunkFactory();

        static std::unique_ptr<Chunk> make(const std::string& id);
    };
}
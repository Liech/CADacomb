#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "CADacomb/Volume/MagicaVox/IO/Rotation.h"

namespace MagicaVoxImporter
{
    class Chunk;

    class Reader
    {
      public:
        Reader();
        Reader(const std::vector<unsigned char>& buffer);
        virtual ~Reader();

        Reader                             read(size_t numberOfBytes);
        std::vector<unsigned char>         readChars(size_t numberOfBytes);
        std::string                        readChunkID();
        int                                readInt();
        std::unique_ptr<Chunk>             readChunk();
        bool                               endOfBufferReached();
        std::string                        readString();
        std::map<std::string, std::string> readDict();
        Rotation                           readRotation();

      private:
        void assertCanRead();

        size_t                     currentPosition = 0;
        std::vector<unsigned char> content;
    };
}

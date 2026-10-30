#pragma once

#include "Chunk.h"
#include <map>

namespace MagicaVoxImporter
{
    // Shape Node Chunk
    class ChunknSHP : public Chunk
    {
      public:
        virtual ~ChunknSHP();

        std::string getID() const override;
        void        read(Reader own, Reader child) override;

        int                                                             id;
        std::map<std::string, std::string>                              attributes;
        std::vector<std::pair<int, std::map<std::string, std::string>>> models;
    };
}
#pragma once

#include "Chunk.h"
#include <map>

namespace MagicaVoxImporter
{
    // Group Node Chunk
    class ChunknGRP : public Chunk
    {
      public:
        virtual ~ChunknGRP();

        std::string getID() const override;
        void        read(Reader own, Reader child) override;

        int                                id;
        std::map<std::string, std::string> attributes;
        std::vector<int>                   childNodes;
    };
}
#pragma once
#include <array>

#include "Chunk.h"

namespace MagicaVoxImporter
{
    // Transform Node Chunk
    class ChunknTRN : public Chunk
    {
      public:
        virtual ~ChunknTRN();

        std::string getID() const override;
        void        read(Reader own, Reader child) override;

        int         id;
        std::string name;
        bool        hidden;
        int         childId;
        int         layerId;
        std::string rotation = "";
        std::string translation;
    };
}
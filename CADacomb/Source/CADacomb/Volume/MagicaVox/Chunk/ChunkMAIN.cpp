#include "ChunkMAIN.h"

#include "CADacomb/Volume/MagicaVox/IO/Writer.h"

namespace MagicaVoxImporter
{
    ChunkMAIN::~ChunkMAIN() {}

    std::string ChunkMAIN::getID() const
    {
        return "MAIN";
    }

    void ChunkMAIN::write(std::vector<unsigned char>& file) const
    {
        std::vector<unsigned char> childContent;

        for (size_t i = 0; i < numberOfChilds(); i++)
        {
            const auto& child = getChild(i);
            child.write(childContent);
        }

        Writer::write(file, getID());
        Writer::write(file, 0);
        Writer::write(file, childContent.size());
        file.insert(file.end(), childContent.begin(), childContent.end());
    }
}
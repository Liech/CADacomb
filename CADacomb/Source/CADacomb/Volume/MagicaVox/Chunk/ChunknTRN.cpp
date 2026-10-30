#include "ChunknTRN.h"
#include "CADacomb/Volume/MagicaVox/IO/Reader.h"
#include <stdexcept>

namespace MagicaVoxImporter
{
    ChunknTRN::~ChunknTRN() {}

    std::string ChunknTRN::getID() const
    {
        return "nTRN";
    }

    void ChunknTRN::read(Reader own, Reader child)
    {
        Chunk::read(own, child);
        id              = own.readInt();
        auto attributes = own.readDict();
        if (attributes.count("_name"))
            name = attributes["_name"];
        else
            name = "Unnamed";
        if (attributes.count("_hidden"))
            hidden = attributes["_hidden"] == "1";
        else
            hidden = false;
        childId = own.readInt();
        own.readInt();
        layerId       = own.readInt();
        int numFrames = own.readInt();
        if (numFrames != 1)
            throw std::runtime_error("Number of Frames must be 1");
        auto frameAttributes = own.readDict();
        if (frameAttributes.count("_t"))
            translation = frameAttributes["_t"];
        if (frameAttributes.count("_r"))
            rotation = frameAttributes["_r"];
    }
}
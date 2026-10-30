#include "ChunknGRP.h"

#include "CADacomb/Volume/MagicaVox/IO/Reader.h"

namespace MagicaVoxImporter
{
    ChunknGRP::~ChunknGRP() {}

    std::string ChunknGRP::getID() const
    {
        return "nGRP";
    }

    void ChunknGRP::read(Reader own, Reader child)
    {
        Chunk::read(own, child);

        id                   = own.readInt();
        attributes           = own.readDict();
        int numberOfChildren = own.readInt();
        childNodes.clear();
        childNodes.resize(numberOfChildren);
        for (int i = 0; i < numberOfChildren; i++)
            childNodes[i] = own.readInt();
    }
}
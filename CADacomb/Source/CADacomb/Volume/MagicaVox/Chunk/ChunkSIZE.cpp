#include "ChunkSIZE.h"

#include <iostream>
#include <string>

#include "CADacomb/Volume/MagicaVox/IO/Reader.h"
#include "CADacomb/Volume/MagicaVox/IO/Writer.h"

namespace MagicaVoxImporter
{
    ChunkSIZE::~ChunkSIZE() {}

    std::string ChunkSIZE::getID() const
    {
        return "SIZE";
    }

    void ChunkSIZE::read(Reader own, Reader child)
    {
        Chunk::read(own, child);
        sizeX = own.readInt();
        sizeY = own.readInt();
        sizeZ = own.readInt();
    }

    void ChunkSIZE::print(int indentation)
    {
        Chunk::print(indentation);
        std::cout << std::string(indentation + 1, ' ') << sizeX << std::endl;
        std::cout << std::string(indentation + 1, ' ') << sizeY << std::endl;
        std::cout << std::string(indentation + 1, ' ') << sizeZ << std::endl;
    }

    void ChunkSIZE::write(std::vector<unsigned char>& file) const
    {
        Writer::write(file, getID());
        Writer::write(file, 12);
        Writer::write(file, 0);
        Writer::write(file, sizeX);
        Writer::write(file, sizeY);
        Writer::write(file, sizeZ);
    }
}
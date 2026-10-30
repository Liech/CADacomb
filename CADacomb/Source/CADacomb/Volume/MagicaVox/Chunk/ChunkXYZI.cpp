#include "ChunkXYZI.h"
#include "CADacomb/Volume/MagicaVox/IO/Reader.h"
#include "CADacomb/Volume/MagicaVox/IO/Writer.h"

#include <iostream>
#include <string>

namespace MagicaVoxImporter
{
    ChunkXYZI::~ChunkXYZI() {}

    std::string ChunkXYZI::getID() const
    {
        return "XYZI";
    }

    void ChunkXYZI::read(Reader own, Reader child)
    {
        Chunk::read(own, child);
        numVoxels = own.readInt();
        content.resize(numVoxels);

        for (int i = 0; i < numVoxels; i++)
        {
            Voxel                      current;
            std::vector<unsigned char> voxelContent = own.readChars(4);
            current.x                               = voxelContent[0];
            current.y                               = voxelContent[1];
            current.z                               = voxelContent[2];
            current.value                           = voxelContent[3];
            content[i]                              = current;
        }
    }

    void ChunkXYZI::print(int indentation)
    {
        Chunk::print(indentation);
        std::cout << std::string(indentation + 1, ' ') << content.size() << std::endl;
    }

    void ChunkXYZI::write(std::vector<unsigned char>& file) const
    {
        Writer::write(file, getID());
        Writer::write(file, 4 + 4 * numVoxels);
        Writer::write(file, 0);
        Writer::write(file, numVoxels);

        for (size_t i = 0; i < numVoxels; i++)
        {
            auto& v = content[i];
            file.push_back(v.x);
            file.push_back(v.y);
            file.push_back(v.z);
            file.push_back(v.value);
        }
    }
}
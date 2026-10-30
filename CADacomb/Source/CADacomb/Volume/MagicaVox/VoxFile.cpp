#include "VoxFile.h"

#include <cassert>
#include <fstream>
#include <memory>

#include "Chunk/ChunkMAIN.h"
#include "Chunk/ChunkMATL.h"
#include "Chunk/ChunkPACK.h"
#include "Chunk/ChunkRGBA.h"
#include "Chunk/ChunkSIZE.h"
#include "Chunk/ChunkXYZI.h"

#include "VoxFileRaw.h"

namespace MagicaVoxImporter
{
    VoxFile::VoxFile(const std::string& filename)
    {
        std::unique_ptr<Chunk> parsedFile     = VoxFileRaw::read(filename);
        size_t                 currentChunk   = 0;
        int                    numberOfModels = 1;
        if (parsedFile->childIsType<ChunkPACK>(currentChunk))
        {
            const ChunkPACK& pack = (const ChunkPACK&)(parsedFile->getChild(currentChunk));
            numberOfModels        = pack.numModels;
            currentChunk++;
        }

        for (int i = 0; i < numberOfModels; i++)
        {
            const ChunkSIZE& size = (const ChunkSIZE&)parsedFile->getChild(currentChunk);
            currentChunk++;
            const ChunkXYZI& xyzi = (const ChunkXYZI&)parsedFile->getChild(currentChunk);
            currentChunk++;
            std::vector<unsigned char> content;
            size_t                     dataSize = (size_t)size.sizeX * (size_t)size.sizeY * (size_t)size.sizeZ;
            content.resize(dataSize);
            for (int j = 0; j < xyzi.numVoxels; j++)
            {
                size_t address   = xyzi.content[j].x + xyzi.content[j].y * size.sizeX + xyzi.content[j].z * size.sizeX * size.sizeY;
                content[address] = xyzi.content[j].value;
            }
            std::array<size_t, 3> dimensions = { size.sizeX, size.sizeY, size.sizeZ };
            models.push_back(std::make_pair(content, dimensions));
        }

        if (!parsedFile->childIsType<ChunkRGBA>(currentChunk))
        {
            auto rgba = ChunkRGBA::getDefault();
            for (int i = 0; i < 256; i++)
                colors[i] = std::array<unsigned char, 4>{ rgba->palette[i].r, rgba->palette[i].g, rgba->palette[i].b, rgba->palette[i].a };
        }
        else
        {
            const ChunkRGBA& rgba = (const ChunkRGBA&)parsedFile->getChild(currentChunk);
            for (int i = 0; i < 256; i++)
                colors[i] = std::array<unsigned char, 4>{ rgba.palette[i].r, rgba.palette[i].g, rgba.palette[i].b, rgba.palette[i].a };
            currentChunk++;
        }

        materials.resize(256);
        while (currentChunk < parsedFile->numberOfChilds())
        {
            if (parsedFile->childIsType<ChunkMATL>(currentChunk))
            {
                const ChunkMATL& mat          = (const ChunkMATL&)parsedFile->getChild(currentChunk);
                materials[mat.materialId - 1] = mat.properties;
            }
            currentChunk++;
        }
    }

    VoxFile::~VoxFile() {}

    std::pair<std::vector<bool>, std::array<size_t, 3>> VoxFile::readBinary(const std::string& filename)
    {
        VoxFile v(filename);
        assert(v.models.size() == 1);
        std::array<size_t, 3> dimensions = v.models[0].second;
        size_t                dataSize   = dimensions[0] * dimensions[1] * dimensions[2];
        std::vector<bool>     content(dataSize);
        for (size_t i = 0; i < dataSize; i++)
            content[i] = v.models[0].first[i] != 0;
        return std::make_pair(std::move(content), dimensions);
    }

    void VoxFile::writeBinary(const std::vector<bool>& data, const std::array<size_t, 3>& size, const std::string& filename)
    {
        ChunkMAIN                  mainChunk;
        std::unique_ptr<ChunkSIZE> sizeChunk = std::make_unique<ChunkSIZE>();
        sizeChunk->sizeX                     = size[0];
        sizeChunk->sizeY                     = size[1];
        sizeChunk->sizeZ                     = size[2];

        std::unique_ptr<ChunkXYZI> contentChunk = std::make_unique<ChunkXYZI>();
        size_t                     dataSize     = size[0] * size[1] * size[2];
        contentChunk->numVoxels                 = dataSize;
        contentChunk->content.resize(dataSize);
        for (size_t i = 0; i < dataSize; i++)
        {
            auto& vox = contentChunk->content[i];
            vox.x     = i % size[0];
            vox.y     = (i / size[0]) % size[1];
            vox.z     = i / (size[0] * size[1]);
            vox.value = data[i] ? 'U' : 0;
        }

        mainChunk.addChild(std::move(sizeChunk));
        mainChunk.addChild(std::move(contentChunk));

        VoxFileRaw::write(mainChunk, filename);
    }
}
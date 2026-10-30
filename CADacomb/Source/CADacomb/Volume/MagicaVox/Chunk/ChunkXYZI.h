#pragma once

#include "CADacomb/Volume/MagicaVox/Chunk/Voxel.h"
#include "Chunk.h"

namespace MagicaVoxImporter
{
    // Chunk id 'XYZI' : model voxels, paired with the SIZE chunk
    class ChunkXYZI : public Chunk
    {
      public:
        virtual ~ChunkXYZI();

        std::string getID() const override;
        void        read(Reader own, Reader child) override;
        void        print(int indentation = 0) override;
        void        write(std::vector<unsigned char>&) const override;

        int                numVoxels = 0;
        std::vector<Voxel> content;
    };
}
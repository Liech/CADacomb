#pragma once

#include "IVec3Compare.h"
#include <glm/glm.hpp>
#include <map>
#include <memory>
#include <vector>

namespace Library
{
    class Triangulation;
    class BinaryVolume;

    class BlockyVoxelTriangulation
    {
      public:
        BlockyVoxelTriangulation(const BinaryVolume& input);
        virtual ~BlockyVoxelTriangulation();

        std::unique_ptr<Triangulation> triangulate();

      private:
        void scan(size_t scanDimension, bool reverse);

        const BinaryVolume&                        input;
        std::map<glm::ivec3, size_t, IVec3Compare> indexMap;
        size_t                                     nextIndex = 0;
        std::vector<size_t>                        indexBuffer;
    };
}

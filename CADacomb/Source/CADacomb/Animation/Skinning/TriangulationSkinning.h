#pragma once

#include "CADacomb/Animation/Skinning/VertexSkinWeights.h"

#include <cstddef>
#include <glm/glm.hpp>
#include <vector>

namespace CADacomb::Animation
{
    // Per-vertex skeletal skinning weights binding triangulation vertices to Skeleton joints.
    class TriangulationSkinning
    {
      public:
        TriangulationSkinning();
        TriangulationSkinning(size_t vertexCount);
        virtual ~TriangulationSkinning();

        size_t getVertexCount() const;
        void   resize(size_t vertexCount);

        const VertexSkinWeights&              getVertexWeights(size_t vertexIndex) const;
        VertexSkinWeights&                    getVertexWeights(size_t vertexIndex);
        void                                  setVertexWeights(size_t vertexIndex, const VertexSkinWeights& weights);
        const std::vector<VertexSkinWeights>& getAllVertexWeights() const;

        void normalizeAll();

        std::vector<glm::dvec3> computeSkinnedPositions(const std::vector<glm::dvec3>& restPositions, const std::vector<glm::dmat4>& skinningMatrices) const;

      private:
        std::vector<VertexSkinWeights> vertexWeights;
    };
}

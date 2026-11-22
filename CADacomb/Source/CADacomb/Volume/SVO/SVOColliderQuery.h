#pragma once

#include "CADacomb/Util/AABB3.h"
#include "CADacomb/Volume/SVO/SVO.h"

#include <cstdint>
#include <functional>
#include <glm/glm.hpp>

namespace CADacomb
{
    // Engine-agnostic octree collision queries for pruning and visiting solid SVO leaf boxes and their persistent node IDs.
    class SVOColliderQuery
    {
      public:
        SVOColliderQuery();
        virtual ~SVOColliderQuery();

        static void queryOverlappingLeaves(const SVO<bool>& svo, const AABB3& svoLocalBounds, const AABB3& queryLocalBounds, const std::function<void(const AABB3&, uint32_t)>& onSolidLeaf);

        static void queryOverlappingLeafPairs(const SVO<bool>&                                                           svoA,
                                              const AABB3&                                                               boundsA,
                                              const SVO<bool>&                                                           svoB,
                                              const AABB3&                                                               boundsB,
                                              const glm::dmat4&                                                          transformBToA,
                                              const std::function<void(const AABB3&, uint32_t, const AABB3&, uint32_t)>& onSolidLeafPair);

      private:
        static glm::ivec3 childOffset(short childIndex, const glm::ivec3& parentOffset, uint32_t parentDimension);
        static void       traverseSingleNodes(const PagedVector<SVONode<bool>>&                  nodes,
                                              uint32_t                                           nodeIdx,
                                              const glm::ivec3&                                  nodeOff,
                                              uint32_t                                           nodeDim,
                                              const glm::dvec3&                                  minCorner,
                                              const glm::dvec3&                                  voxelScale,
                                              const AABB3&                                       queryLocalBounds,
                                              const std::function<void(const AABB3&, uint32_t)>& onSolidLeaf);
        static void       traverseDualNodes(const PagedVector<SVONode<bool>>&                                          nodesA,
                                            uint32_t                                                                   idxA,
                                            const glm::ivec3&                                                          offA,
                                            uint32_t                                                                   dimA,
                                            const glm::dvec3&                                                          minA,
                                            const glm::dvec3&                                                          scaleA,
                                            const PagedVector<SVONode<bool>>&                                          nodesB,
                                            uint32_t                                                                   idxB,
                                            const glm::ivec3&                                                          offB,
                                            uint32_t                                                                   dimB,
                                            const glm::dvec3&                                                          minB,
                                            const glm::dvec3&                                                          scaleB,
                                            const glm::dmat4&                                                          transformBToA,
                                            const glm::dmat4&                                                          transformAToB,
                                            const std::function<void(const AABB3&, uint32_t, const AABB3&, uint32_t)>& onSolidLeafPair);
    };
}

#include "CADacomb/Volume/SVO/SVOColliderQuery.h"

namespace CADacomb
{
    SVOColliderQuery::SVOColliderQuery() {}

    SVOColliderQuery::~SVOColliderQuery() {}

    void SVOColliderQuery::queryOverlappingLeaves(const SVO<bool>& svo, const AABB3& svoLocalBounds, const AABB3& queryLocalBounds, const std::function<void(const AABB3&, uint32_t)>& onSolidLeaf)
    {
        uint32_t dimension = svo.getDimension();
        if (dimension == 0 || svo.getRaw().empty() || !svoLocalBounds.intersects(queryLocalBounds))
        {
            return;
        }

        glm::dvec3 voxelScale = svoLocalBounds.getSize() / static_cast<double>(dimension);
        traverseSingleNodes(svo.getRaw(), 0, glm::ivec3(0), dimension, svoLocalBounds.getMin(), voxelScale, queryLocalBounds, onSolidLeaf);
    }

    void SVOColliderQuery::queryOverlappingLeafPairs(const SVO<bool>&                                                           svoA,
                                                     const AABB3&                                                               boundsA,
                                                     const SVO<bool>&                                                           svoB,
                                                     const AABB3&                                                               boundsB,
                                                     const glm::dmat4&                                                          transformBToA,
                                                     const std::function<void(const AABB3&, uint32_t, const AABB3&, uint32_t)>& onSolidLeafPair)
    {
        uint32_t dimA = svoA.getDimension();
        uint32_t dimB = svoB.getDimension();
        if (dimA == 0 || dimB == 0 || svoA.getRaw().empty() || svoB.getRaw().empty())
        {
            return;
        }

        glm::dmat4 transformAToB = glm::inverse(transformBToA);
        glm::dvec3 scaleA        = boundsA.getSize() / static_cast<double>(dimA);
        glm::dvec3 scaleB        = boundsB.getSize() / static_cast<double>(dimB);

        traverseDualNodes(
          svoA.getRaw(), 0, glm::ivec3(0), dimA, boundsA.getMin(), scaleA, svoB.getRaw(), 0, glm::ivec3(0), dimB, boundsB.getMin(), scaleB, transformBToA, transformAToB, onSolidLeafPair);
    }

    glm::ivec3 SVOColliderQuery::childOffset(short childIndex, const glm::ivec3& parentOffset, uint32_t parentDimension)
    {
        uint32_t   half = parentDimension / 2;
        glm::ivec3 next = parentOffset;
        if (childIndex & 1)
        {
            next.x += static_cast<int>(half);
        }
        if (childIndex & 2)
        {
            next.y += static_cast<int>(half);
        }
        if (childIndex & 4)
        {
            next.z += static_cast<int>(half);
        }
        return next;
    }

    void SVOColliderQuery::traverseSingleNodes(const PagedVector<SVONode<bool>>&                  nodes,
                                               uint32_t                                           nodeIdx,
                                               const glm::ivec3&                                  nodeOff,
                                               uint32_t                                           nodeDim,
                                               const glm::dvec3&                                  minCorner,
                                               const glm::dvec3&                                  voxelScale,
                                               const AABB3&                                       queryLocalBounds,
                                               const std::function<void(const AABB3&, uint32_t)>& onSolidLeaf)
    {
        const auto& node   = nodes[nodeIdx];
        bool        isLeaf = (node.childPointer == 0xFFFFFFFF);
        if (isLeaf && !node.value)
        {
            return;
        }

        AABB3 nodeBox(minCorner + glm::dvec3(nodeOff) * voxelScale, minCorner + glm::dvec3(nodeOff + glm::ivec3(static_cast<int>(nodeDim))) * voxelScale);
        if (!nodeBox.intersects(queryLocalBounds))
        {
            return;
        }

        if (isLeaf)
        {
            onSolidLeaf(nodeBox, nodeIdx);
            return;
        }

        uint32_t half = nodeDim / 2;
        for (short i = 0; i < 8; i++)
        {
            glm::ivec3 cOff = childOffset(i, nodeOff, nodeDim);
            traverseSingleNodes(nodes, node.childPointer + i, cOff, half, minCorner, voxelScale, queryLocalBounds, onSolidLeaf);
        }
    }

    void SVOColliderQuery::traverseDualNodes(const PagedVector<SVONode<bool>>&                                          nodesA,
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
                                             const std::function<void(const AABB3&, uint32_t, const AABB3&, uint32_t)>& onSolidLeafPair)
    {
        const auto& nodeA = nodesA[idxA];
        const auto& nodeB = nodesB[idxB];
        bool        leafA = (nodeA.childPointer == 0xFFFFFFFF);
        bool        leafB = (nodeB.childPointer == 0xFFFFFFFF);

        if ((leafA && !nodeA.value) || (leafB && !nodeB.value))
        {
            return;
        }

        AABB3 boxA(minA + glm::dvec3(offA) * scaleA, minA + glm::dvec3(offA + glm::ivec3(static_cast<int>(dimA))) * scaleA);
        AABB3 boxB(minB + glm::dvec3(offB) * scaleB, minB + glm::dvec3(offB + glm::ivec3(static_cast<int>(dimB))) * scaleB);

        if (!boxA.intersects(boxB.transformed(transformBToA)) || !boxB.intersects(boxA.transformed(transformAToB)))
        {
            return;
        }

        if (leafA && leafB)
        {
            onSolidLeafPair(boxA, idxA, boxB, idxB);
            return;
        }

        bool splitA = !leafA && (leafB || (static_cast<double>(dimA) * scaleA.x >= static_cast<double>(dimB) * scaleB.x));
        if (splitA)
        {
            uint32_t halfA = dimA / 2;
            for (short i = 0; i < 8; i++)
            {
                glm::ivec3 cOffA = childOffset(i, offA, dimA);
                traverseDualNodes(nodesA, nodeA.childPointer + i, cOffA, halfA, minA, scaleA, nodesB, idxB, offB, dimB, minB, scaleB, transformBToA, transformAToB, onSolidLeafPair);
            }
        }
        else
        {
            uint32_t halfB = dimB / 2;
            for (short i = 0; i < 8; i++)
            {
                glm::ivec3 cOffB = childOffset(i, offB, dimB);
                traverseDualNodes(nodesA, idxA, offA, dimA, minA, scaleA, nodesB, nodeB.childPointer + i, cOffB, halfB, minB, scaleB, transformBToA, transformAToB, onSolidLeafPair);
            }
        }
    }
}

#ifdef ISTESTPROJECT
#include <catch2/catch_test_macros.hpp>
#include <glm/gtc/matrix_transform.hpp>

TEST_CASE("SVOColliderQuery/PrunesEmptyAndNonOverlappingBranches")
{
    CADacomb::SVO<bool> svo(16, false);
    svo.setVoxel(glm::ivec3(2, 2, 2), true);
    svo.setVoxel(glm::ivec3(14, 14, 14), true);

    CADacomb::AABB3              bounds(glm::dvec3(0.0), glm::dvec3(16.0));
    CADacomb::AABB3              query(glm::dvec3(1.5), glm::dvec3(2.5));
    std::vector<CADacomb::AABB3> hits;

    CADacomb::SVOColliderQuery::queryOverlappingLeaves(svo, bounds, query, [&hits](const CADacomb::AABB3& leafBox, uint32_t) { hits.push_back(leafBox); });

    REQUIRE(hits.size() == 1);
    CHECK(hits[0].getMin() == glm::dvec3(2.0, 2.0, 2.0));
    CHECK(hits[0].getMax() == glm::dvec3(3.0, 3.0, 3.0));
}

TEST_CASE("SVOColliderQuery/DualSVOOverlappingLeafPairsAndDynamicUpdate")
{
    CADacomb::SVO<bool> svoA(8, false);
    svoA.setVoxel(glm::ivec3(4, 4, 4), true);
    CADacomb::AABB3 boundsA(glm::dvec3(0.0), glm::dvec3(8.0));

    CADacomb::SVO<bool> svoB(8, false);
    svoB.setVoxel(glm::ivec3(0, 0, 0), true);
    CADacomb::AABB3 boundsB(glm::dvec3(0.0), glm::dvec3(8.0));

    glm::dmat4 transformBToA = glm::translate(glm::dmat4(1.0), glm::dvec3(3.5, 3.5, 3.5));

    size_t pairCount = 0;
    CADacomb::SVOColliderQuery::queryOverlappingLeafPairs(
      svoA, boundsA, svoB, boundsB, transformBToA, [&pairCount](const CADacomb::AABB3&, uint32_t, const CADacomb::AABB3&, uint32_t) { pairCount++; });
    CHECK(pairCount == 1);

    svoA.setVoxel(glm::ivec3(4, 4, 4), false);
    pairCount = 0;
    CADacomb::SVOColliderQuery::queryOverlappingLeafPairs(
      svoA, boundsA, svoB, boundsB, transformBToA, [&pairCount](const CADacomb::AABB3&, uint32_t, const CADacomb::AABB3&, uint32_t) { pairCount++; });
    CHECK(pairCount == 0);
}
#endif

#pragma once

#include "SVO.h"
#include "SVOAxis.h"
#include "SVODualNode.h"
#include <array>
#include <concepts>
#include <cstdint>
#include <glm/glm.hpp>
#include <vector>

namespace CADacomb
{
    class SVODualTraversal
    {
      public:
        SVODualTraversal();
        virtual ~SVODualTraversal();

        template<typename T>
        static SVODualNode<T> findSubtreeNode(const SVO<T>& svo, const glm::ivec3& offset, uint32_t subDimension, const T& virtualValue = T{});

        template<typename T, typename LeafPredicateA, typename LeafPredicateB, typename FaceCallback>
        static void traverseFacePair(const SVO<T>&         svo,
                                     const SVODualNode<T>& rootA,
                                     const SVODualNode<T>& rootB,
                                     SVOAxis               axis,
                                     LeafPredicateA&&      isLeafA,
                                     LeafPredicateB&&      isLeafB,
                                     FaceCallback&&        onFace);

        template<typename T, typename LeafPredicate, typename NodeCallback, typename FaceCallback>
        static void traverseSubtree(const SVO<T>& svo, const SVODualNode<T>& rootNode, LeafPredicate&& isLeaf, NodeCallback&& onNode, FaceCallback&& onFace);

        template<typename T, typename LeafPredicate, typename NodeCallback, typename FaceCallback>
        static void traverse(const SVO<T>& svo, LeafPredicate&& isLeaf, NodeCallback&& onNode, FaceCallback&& onFace, const T& boundaryValue = T{});

      private:
        template<typename T>
        static SVODualNode<T> getChild(const std::vector<SVONode<T>>& rawNodes, const SVODualNode<T>& parent, int childBit);

        static int getPosChild(SVOAxis ax, int k);
        static int getNegChild(SVOAxis ax, int k);
    };

    template<typename T>
    SVODualNode<T> SVODualTraversal::findSubtreeNode(const SVO<T>& svo, const glm::ivec3& offset, uint32_t subDimension, const T& virtualValue)
    {
        uint32_t    dimension = svo.getDimension();
        const auto& rawNodes  = svo.getRaw();
        if (dimension == 0 || rawNodes.empty() || subDimension == 0 || offset.x < 0 || offset.y < 0 || offset.z < 0 || offset.x + static_cast<int>(subDimension) > static_cast<int>(dimension) ||
            offset.y + static_cast<int>(subDimension) > static_cast<int>(dimension) || offset.z + static_cast<int>(subDimension) > static_cast<int>(dimension))
        {
            return SVODualNode<T>{ .nodeIndex = 0xFFFFFFFF, .offset = offset, .size = subDimension, .depth = 0, .value = virtualValue, .isLeaf = true, .isVirtual = true };
        }

        uint32_t   curNodeIdx = 0;
        glm::ivec3 curOffset(0);
        uint32_t   curSize  = dimension;
        int        curDepth = 0;

        while (curSize > subDimension)
        {
            const auto& raw = rawNodes[curNodeIdx];
            if (raw.childPointer == 0xFFFFFFFF)
            {
                return SVODualNode<T>{ .nodeIndex = curNodeIdx, .offset = offset, .size = subDimension, .depth = curDepth, .value = raw.value, .isLeaf = true, .isVirtual = false };
            }

            uint32_t half     = curSize / 2;
            int      childBit = 0;
            if (offset.x >= curOffset.x + static_cast<int>(half))
            {
                childBit |= 1;
                curOffset.x += static_cast<int>(half);
            }
            if (offset.y >= curOffset.y + static_cast<int>(half))
            {
                childBit |= 2;
                curOffset.y += static_cast<int>(half);
            }
            if (offset.z >= curOffset.z + static_cast<int>(half))
            {
                childBit |= 4;
                curOffset.z += static_cast<int>(half);
            }

            curNodeIdx = raw.childPointer + childBit;
            curSize    = half;
            curDepth++;
        }

        const auto& finalRaw = rawNodes[curNodeIdx];
        return SVODualNode<T>{
            .nodeIndex = curNodeIdx, .offset = offset, .size = subDimension, .depth = curDepth, .value = finalRaw.value, .isLeaf = (finalRaw.childPointer == 0xFFFFFFFF), .isVirtual = false
        };
    }

    template<typename T, typename LeafPredicateA, typename LeafPredicateB, typename FaceCallback>
    void SVODualTraversal::traverseFacePair(const SVO<T>&         svo,
                                            const SVODualNode<T>& rootA,
                                            const SVODualNode<T>& rootB,
                                            SVOAxis               axis,
                                            LeafPredicateA&&      isLeafA,
                                            LeafPredicateB&&      isLeafB,
                                            FaceCallback&&        onFace)
    {
        const auto& rawNodes = svo.getRaw();
        if (svo.getDimension() == 0 || rawNodes.empty())
        {
            return;
        }

        auto traverseFace = [&](auto& selfFace, const SVODualNode<T>& nodeA, const SVODualNode<T>& nodeB, SVOAxis ax) -> void
        {
            bool aIsLeaf = nodeA.isVirtual || nodeA.isLeaf || isLeafA(nodeA);
            bool bIsLeaf = nodeB.isVirtual || nodeB.isLeaf || isLeafB(nodeB);

            if (aIsLeaf && bIsLeaf)
            {
                onFace(nodeA, nodeB, ax);
                return;
            }

            if (!aIsLeaf && !bIsLeaf)
            {
                for (int k = 0; k < 4; k++)
                {
                    selfFace(selfFace, getChild(rawNodes, nodeA, getPosChild(ax, k)), getChild(rawNodes, nodeB, getNegChild(ax, k)), ax);
                }
                return;
            }

            if (!aIsLeaf && bIsLeaf)
            {
                for (int k = 0; k < 4; k++)
                {
                    selfFace(selfFace, getChild(rawNodes, nodeA, getPosChild(ax, k)), nodeB, ax);
                }
                return;
            }

            for (int k = 0; k < 4; k++)
            {
                selfFace(selfFace, nodeA, getChild(rawNodes, nodeB, getNegChild(ax, k)), ax);
            }
        };

        traverseFace(traverseFace, rootA, rootB, axis);
    }

    template<typename T, typename LeafPredicate, typename NodeCallback, typename FaceCallback>
    void SVODualTraversal::traverseSubtree(const SVO<T>& svo, const SVODualNode<T>& rootNode, LeafPredicate&& isLeaf, NodeCallback&& onNode, FaceCallback&& onFace)
    {
        const auto& rawNodes = svo.getRaw();
        if (svo.getDimension() == 0 || rawNodes.empty() || rootNode.isVirtual)
        {
            return;
        }

        auto traverseNode = [&](auto& selfNode, const SVODualNode<T>& node) -> void
        {
            if (isLeaf(node) || node.isLeaf)
            {
                SVODualNode<T> leafNode = node;
                leafNode.isLeaf         = true;
                onNode(leafNode);
                return;
            }

            const auto& rawNode   = rawNodes[node.nodeIndex];
            uint32_t    half      = node.size / 2;
            int         nextDepth = node.depth + 1;

            std::array<SVODualNode<T>, 8> children;
            for (int i = 0; i < 8; i++)
            {
                uint32_t    childIdx = rawNode.childPointer + i;
                const auto& cRaw     = rawNodes[childIdx];
                glm::ivec3  cOffset  = node.offset + glm::ivec3((i & 1) ? half : 0, (i & 2) ? half : 0, (i & 4) ? half : 0);
                children[i] =
                  SVODualNode<T>{ .nodeIndex = childIdx, .offset = cOffset, .size = half, .depth = nextDepth, .value = cRaw.value, .isLeaf = (cRaw.childPointer == 0xFFFFFFFF), .isVirtual = false };
            }

            for (int i = 0; i < 8; i++)
            {
                selfNode(selfNode, children[i]);
            }

            for (int k = 0; k < 4; k++)
            {
                int u = (k & 1) ? 2 : 0;
                int v = (k & 2) ? 4 : 0;
                traverseFacePair(svo, children[0 | u | v], children[1 | u | v], SVOAxis::X, isLeaf, isLeaf, onFace);
            }
            for (int k = 0; k < 4; k++)
            {
                int u = (k & 1) ? 1 : 0;
                int v = (k & 2) ? 4 : 0;
                traverseFacePair(svo, children[0 | u | v], children[2 | u | v], SVOAxis::Y, isLeaf, isLeaf, onFace);
            }
            for (int k = 0; k < 4; k++)
            {
                int u = (k & 1) ? 1 : 0;
                int v = (k & 2) ? 2 : 0;
                traverseFacePair(svo, children[0 | u | v], children[4 | u | v], SVOAxis::Z, isLeaf, isLeaf, onFace);
            }
        };

        traverseNode(traverseNode, rootNode);
    }

    template<typename T, typename LeafPredicate, typename NodeCallback, typename FaceCallback>
    void SVODualTraversal::traverse(const SVO<T>& svo, LeafPredicate&& isLeaf, NodeCallback&& onNode, FaceCallback&& onFace, const T& boundaryValue)
    {
        uint32_t    dimension = svo.getDimension();
        const auto& rawNodes  = svo.getRaw();
        if (dimension == 0 || rawNodes.empty())
        {
            return;
        }

        SVODualNode<T> rootNode{
            .nodeIndex = 0, .offset = glm::ivec3(0), .size = dimension, .depth = 0, .value = rawNodes[0].value, .isLeaf = (rawNodes[0].childPointer == 0xFFFFFFFF), .isVirtual = false
        };

        traverseSubtree(svo, rootNode, isLeaf, onNode, onFace);

        auto makeVirtual = [&](const glm::ivec3& off) -> SVODualNode<T>
        { return SVODualNode<T>{ .nodeIndex = 0xFFFFFFFF, .offset = off, .size = dimension, .depth = 0, .value = boundaryValue, .isLeaf = true, .isVirtual = true }; };

        traverseFacePair(svo, makeVirtual(glm::ivec3(-int(dimension), 0, 0)), rootNode, SVOAxis::X, isLeaf, isLeaf, onFace);
        traverseFacePair(svo, rootNode, makeVirtual(glm::ivec3(int(dimension), 0, 0)), SVOAxis::X, isLeaf, isLeaf, onFace);

        traverseFacePair(svo, makeVirtual(glm::ivec3(0, -int(dimension), 0)), rootNode, SVOAxis::Y, isLeaf, isLeaf, onFace);
        traverseFacePair(svo, rootNode, makeVirtual(glm::ivec3(0, int(dimension), 0)), SVOAxis::Y, isLeaf, isLeaf, onFace);

        traverseFacePair(svo, makeVirtual(glm::ivec3(0, 0, -int(dimension))), rootNode, SVOAxis::Z, isLeaf, isLeaf, onFace);
        traverseFacePair(svo, rootNode, makeVirtual(glm::ivec3(0, 0, int(dimension))), SVOAxis::Z, isLeaf, isLeaf, onFace);
    }

    template<typename T>
    SVODualNode<T> SVODualTraversal::getChild(const std::vector<SVONode<T>>& rawNodes, const SVODualNode<T>& parent, int childBit)
    {
        uint32_t   half    = parent.size / 2;
        glm::ivec3 cOffset = parent.offset + glm::ivec3((childBit & 1) ? half : 0, (childBit & 2) ? half : 0, (childBit & 4) ? half : 0);

        if (parent.isVirtual)
        {
            return SVODualNode<T>{ .nodeIndex = 0xFFFFFFFF, .offset = cOffset, .size = half, .depth = parent.depth + 1, .value = parent.value, .isLeaf = true, .isVirtual = true };
        }

        uint32_t    childIdx = rawNodes[parent.nodeIndex].childPointer + childBit;
        const auto& cRaw     = rawNodes[childIdx];
        return SVODualNode<T>{
            .nodeIndex = childIdx, .offset = cOffset, .size = half, .depth = parent.depth + 1, .value = cRaw.value, .isLeaf = (cRaw.childPointer == 0xFFFFFFFF), .isVirtual = false
        };
    }
}

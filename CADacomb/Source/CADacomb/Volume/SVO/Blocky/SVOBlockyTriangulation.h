#pragma once

#include "CADacomb/Triangle/Triangulation.h"
#include "CADacomb/Volume/SVO/SVO.h"
#include "CADacomb/Volume/SVO/SVODualTraversal.h"
#include "SVOBlockyOptions.h"
#include <algorithm>
#include <cmath>
#include <glm/glm.hpp>
#include <memory>
#include <vector>

namespace CADacomb
{
    class SVOBlockyTriangulation
    {
      public:
        SVOBlockyTriangulation();
        virtual ~SVOBlockyTriangulation();

        template<typename T>
        static std::unique_ptr<Triangulation> triangulate(const SVO<T>& svo, const SVOBlockyOptions& options = {});
    };

    template<typename T>
    std::unique_ptr<Triangulation> SVOBlockyTriangulation::triangulate(const SVO<T>& svo, const SVOBlockyOptions& options)
    {
        auto result = std::make_unique<Triangulation>();

        uint32_t    dimension = svo.getDimension();
        const auto& rawNodes  = svo.getRaw();
        if (dimension == 0 || rawNodes.empty())
        {
            return result;
        }

        uint32_t treeMaxDepth = 0;
        while ((1u << treeMaxDepth) < dimension)
        {
            treeMaxDepth++;
        }

        int effectiveMaxDepth = (options.maxDepth > 0) ? std::min(static_cast<int>(treeMaxDepth), options.maxDepth) : static_cast<int>(treeMaxDepth);
        int effectiveMinDepth = std::clamp(options.minDepth, 1, effectiveMaxDepth);

        auto isSolid = [](const T& val) -> bool
        {
            if constexpr (std::is_same_v<T, bool>)
            {
                return val;
            }
            else if constexpr (requires { val.fill; })
            {
                return val.fill > 0;
            }
            else if constexpr (requires { val > 0; })
            {
                return val > 0;
            }
            else
            {
                return static_cast<bool>(val);
            }
        };

        auto addQuad = [&](const glm::dvec3& v0, const glm::dvec3& v1, const glm::dvec3& v2, const glm::dvec3& v3)
        {
            size_t baseIdx = result->vertices.size();
            result->vertices.push_back(v0);
            result->vertices.push_back(v1);
            result->vertices.push_back(v2);
            result->vertices.push_back(v3);

            result->indices.push_back(baseIdx + 0);
            result->indices.push_back(baseIdx + 1);
            result->indices.push_back(baseIdx + 2);

            result->indices.push_back(baseIdx + 0);
            result->indices.push_back(baseIdx + 2);
            result->indices.push_back(baseIdx + 3);
        };

        auto emitBlockFaces = [&](const glm::dvec3& minB, const glm::dvec3& maxB, const bool exposedFaces[6])
        {
            if (exposedFaces[0])
            {
                addQuad(glm::dvec3(minB.x, minB.y, minB.z), glm::dvec3(minB.x, minB.y, maxB.z), glm::dvec3(minB.x, maxB.y, maxB.z), glm::dvec3(minB.x, maxB.y, minB.z));
            }
            if (exposedFaces[1])
            {
                addQuad(glm::dvec3(maxB.x, minB.y, minB.z), glm::dvec3(maxB.x, maxB.y, minB.z), glm::dvec3(maxB.x, maxB.y, maxB.z), glm::dvec3(maxB.x, minB.y, maxB.z));
            }
            if (exposedFaces[2])
            {
                addQuad(glm::dvec3(minB.x, minB.y, minB.z), glm::dvec3(maxB.x, minB.y, minB.z), glm::dvec3(maxB.x, minB.y, maxB.z), glm::dvec3(minB.x, minB.y, maxB.z));
            }
            if (exposedFaces[3])
            {
                addQuad(glm::dvec3(minB.x, maxB.y, minB.z), glm::dvec3(minB.x, maxB.y, maxB.z), glm::dvec3(maxB.x, maxB.y, maxB.z), glm::dvec3(maxB.x, maxB.y, minB.z));
            }
            if (exposedFaces[4])
            {
                addQuad(glm::dvec3(minB.x, minB.y, minB.z), glm::dvec3(minB.x, maxB.y, minB.z), glm::dvec3(maxB.x, maxB.y, minB.z), glm::dvec3(maxB.x, minB.y, minB.z));
            }
            if (exposedFaces[5])
            {
                addQuad(glm::dvec3(minB.x, minB.y, maxB.z), glm::dvec3(maxB.x, minB.y, maxB.z), glm::dvec3(maxB.x, maxB.y, maxB.z), glm::dvec3(minB.x, maxB.y, maxB.z));
            }
        };

        auto computeT = [&](double dist) -> double
        {
            if (options.lodFarDistance > options.lodNearDistance)
            {
                return std::clamp((dist - options.lodNearDistance) / (options.lodFarDistance - options.lodNearDistance), 0.0, 1.0);
            }
            return std::clamp(dist / std::max(1e-4, options.lodDistance), 0.0, 1.0);
        };

        int uniformTargetDepth = options.targetDepth;
        if (uniformTargetDepth < 0 || uniformTargetDepth > static_cast<int>(treeMaxDepth))
        {
            uniformTargetDepth = static_cast<int>(treeMaxDepth);
        }

        auto isLeaf = [&](const SVODualNode<T>& node) -> bool
        {
            if (node.isLeaf)
            {
                return true;
            }

            if (!options.adaptive)
            {
                return node.depth >= uniformTargetDepth;
            }

            glm::dvec3 normCenter   = (glm::dvec3(node.offset) + 0.5 * double(node.size)) / double(dimension);
            glm::dvec3 worldCenter  = options.origin + normCenter * options.size;
            double     dist         = glm::distance(worldCenter, options.focalPoint);
            double     t            = computeT(dist);
            int        desiredDepth = static_cast<int>(std::round(double(effectiveMaxDepth) * (1.0 - t) + double(effectiveMinDepth) * t));
            return node.depth >= desiredDepth;
        };

        struct ActiveLeaf
        {
            glm::ivec3 offset;
            uint32_t   size;
            bool       solid;
        };

        std::vector<ActiveLeaf> activeLeaves;
        activeLeaves.reserve(rawNodes.size() / 4);

        std::vector<uint32_t> nodeToLeaf(rawNodes.size(), 0xFFFFFFFF);
        std::vector<uint8_t>  exposedFaces(rawNodes.size(), 0);

        auto onNode = [&](const SVODualNode<T>& leaf)
        {
            if (leaf.isVirtual)
                return;

            bool     solid   = isSolid(leaf.value);
            uint32_t leafIdx = static_cast<uint32_t>(activeLeaves.size());
            activeLeaves.push_back({ leaf.offset, leaf.size, solid });
            nodeToLeaf[leaf.nodeIndex] = leafIdx;
        };

        auto onFace = [&](const SVODualNode<T>& nodeA, const SVODualNode<T>& nodeB, SVOAxis axis)
        {
            bool solidA = !nodeA.isVirtual && isSolid(nodeA.value);
            bool solidB = !nodeB.isVirtual && isSolid(nodeB.value);

            bool materialDiff = (solidA != solidB);
            if constexpr (requires { nodeA.value != nodeB.value; })
            {
                if (solidA && solidB && (nodeA.value != nodeB.value))
                {
                    materialDiff = true;
                }
            }

            if (!materialDiff)
            {
                return;
            }

            if (solidA && !nodeA.isVirtual)
            {
                int faceIdx = 2 * static_cast<int>(axis) + 1;
                exposedFaces[nodeA.nodeIndex] |= (1 << faceIdx);
            }

            if (solidB && !nodeB.isVirtual)
            {
                int faceIdx = 2 * static_cast<int>(axis) + 0;
                exposedFaces[nodeB.nodeIndex] |= (1 << faceIdx);
            }
        };

        SVODualTraversal::traverse(svo, isLeaf, onNode, onFace);

        for (size_t nodeIdx = 0; nodeIdx < rawNodes.size(); ++nodeIdx)
        {
            uint8_t mask = exposedFaces[nodeIdx];
            if (mask == 0)
                continue;

            uint32_t leafIdx = nodeToLeaf[nodeIdx];
            if (leafIdx == 0xFFFFFFFF)
                continue;

            const auto& blk = activeLeaves[leafIdx];
            if (!blk.solid)
                continue;

            glm::dvec3 minB = options.origin + (glm::dvec3(blk.offset) / double(dimension)) * options.size;
            glm::dvec3 maxB = minB + (glm::dvec3(blk.size) / double(dimension)) * options.size;

            bool exp[6];
            for (int f = 0; f < 6; ++f)
            {
                exp[f] = (mask & (1 << f)) != 0;
            }
            emitBlockFaces(minB, maxB, exp);
        }

        return result;
    }
}

#pragma once

#include "CADacomb/Triangle/Triangulation.h"
#include "CADacomb/Volume/SVO/SVO.h"
#include "CADacomb/Volume/SVO/SVODualTraversal.h"
#include "MeshChunk.h"
#include "PackedQuad.h"
#include "SVOMesherOptions.h"
#include <algorithm>
#include <cmath>
#include <glm/glm.hpp>
#include <memory>
#include <type_traits>
#include <vector>

#if defined(_OPENMP)
#include <omp.h>
#endif

namespace CADacomb
{
    class SVOCPUMesher
    {
      public:
        SVOCPUMesher();
        virtual ~SVOCPUMesher();

        template<typename T>
        static std::vector<MeshChunk> generateChunks(const SVO<T>& svo, const SVOMesherOptions& options);

        static void generateBorderChunks(std::vector<MeshChunk>& chunks);

        template<typename T = bool>
        static void meshChunk(const SVO<T>& svo, MeshChunk& chunk);

        template<typename T>
        static void meshChunks(const SVO<T>& svo, std::vector<MeshChunk>& chunks);

        static std::unique_ptr<CADacomb::Triangulation> extractTriangulation(const std::vector<MeshChunk>& chunks, const glm::dvec3& worldOrigin = glm::dvec3(0.0), double voxelWorldSize = 1.0);
        static std::unique_ptr<CADacomb::Triangulation> extractTriangulation(const MeshChunk& chunk, const glm::dvec3& worldOrigin = glm::dvec3(0.0), double voxelWorldSize = 1.0);

        template<typename T>
        static std::unique_ptr<CADacomb::Triangulation> triangulate(const SVO<T>& svo, const SVOMesherOptions& options);
    };

    template<typename T>
    std::vector<MeshChunk> SVOCPUMesher::generateChunks(const SVO<T>& svo, const SVOMesherOptions& options)
    {
        uint32_t masterDim = svo.getDimension();
        if (masterDim == 0)
        {
            return {};
        }

        int baseChunkSize = std::clamp(options.chunkSize, 8, static_cast<int>(masterDim));
        int p2            = 1;
        while (p2 < baseChunkSize)
        {
            p2 <<= 1;
        }
        baseChunkSize = std::min(p2, static_cast<int>(masterDim));

        int maxRes = std::max(8, options.maxResolution);

        std::vector<MeshChunk> chunks;
        for (int z = 0; z < static_cast<int>(masterDim); z += baseChunkSize)
        {
            for (int y = 0; y < static_cast<int>(masterDim); y += baseChunkSize)
            {
                for (int x = 0; x < static_cast<int>(masterDim); x += baseChunkSize)
                {
                    MeshChunk chunk;
                    chunk.minBound      = glm::ivec3(x, y, z);
                    chunk.size          = baseChunkSize;
                    chunk.maxResolution = maxRes;

                    if (options.adaptive)
                    {
                        glm::dvec3 normCenter  = (glm::dvec3(chunk.minBound) + 0.5 * double(baseChunkSize)) / double(masterDim);
                        glm::dvec3 worldCenter = options.origin + normCenter * options.size;
                        double     dist        = glm::distance(worldCenter, options.focalPoint);

                        double t = 0.0;
                        if (options.lodFarDistance > options.lodNearDistance)
                        {
                            t = std::clamp((dist - options.lodNearDistance) / (options.lodFarDistance - options.lodNearDistance), 0.0, 1.0);
                        }
                        else
                        {
                            t = std::clamp(dist / std::max(1e-4, options.lodFarDistance), 0.0, 1.0);
                        }

                        int targetLOD  = static_cast<int>(std::round(double(options.minLOD) * (1.0 - t) + double(options.maxLOD) * t));
                        chunk.lodLevel = std::clamp(targetLOD, options.minLOD, options.maxLOD);
                    }
                    else
                    {
                        chunk.lodLevel = 0;
                    }

                    chunks.push_back(chunk);
                }
            }
        }
        return chunks;
    }

    template<typename T>
    void SVOCPUMesher::meshChunk(const SVO<T>& svo, MeshChunk& chunk)
    {
        chunk.quads.clear();
        chunk.evaluatedCellCount = 0;

        uint32_t masterDim = svo.getDimension();
        if (masterDim == 0 || chunk.size == 0)
        {
            return;
        }

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

        auto getMatId = [](const T& val) -> uint32_t
        {
            if constexpr (std::is_integral_v<T>)
            {
                return static_cast<uint32_t>(val);
            }
            else
            {
                return 1u;
            }
        };

        auto getScaleExp = [](uint32_t scale) -> uint32_t
        {
            uint32_t exp = 0;
            while (scale >>= 1)
            {
                exp++;
            }
            return exp;
        };

        auto emitFaceQuads = [&](const SVODualNode<T>& nodeA, const SVODualNode<T>& nodeB, SVOAxis axis, bool allowPositiveDir, bool allowNegativeDir) -> void
        {
            bool solidA = !nodeA.isVirtual && isSolid(nodeA.value);
            bool solidB = !nodeB.isVirtual && isSolid(nodeB.value);
            if (!solidA && !solidB)
            {
                return;
            }
            if (solidA && solidB && nodeA.value == nodeB.value)
            {
                return;
            }

            uint32_t   s             = std::min(nodeA.size, nodeB.size);
            uint32_t   scaleExp      = getScaleExp(s);
            int        axisIdx       = static_cast<int>(axis);
            glm::ivec3 smallerOffset = (nodeA.size <= nodeB.size) ? nodeA.offset : nodeB.offset;
            int        planeCoord    = nodeB.offset[axisIdx];

            if (solidA && allowPositiveDir)
            {
                glm::ivec3 localPos = smallerOffset - chunk.minBound;
                localPos[axisIdx]   = (planeCoord - static_cast<int>(s)) - chunk.minBound[axisIdx];
                uint32_t dir        = static_cast<uint32_t>(2 * axisIdx + 1);
                chunk.quads.push_back(
                  PackedQuad::create(static_cast<uint32_t>(localPos.x), static_cast<uint32_t>(localPos.y), static_cast<uint32_t>(localPos.z), dir, scaleExp, getMatId(nodeA.value)));
            }

            if (solidB && allowNegativeDir)
            {
                glm::ivec3 localPos = smallerOffset - chunk.minBound;
                localPos[axisIdx]   = planeCoord - chunk.minBound[axisIdx];
                uint32_t dir        = static_cast<uint32_t>(2 * axisIdx + 0);
                chunk.quads.push_back(
                  PackedQuad::create(static_cast<uint32_t>(localPos.x), static_cast<uint32_t>(localPos.y), static_cast<uint32_t>(localPos.z), dir, scaleExp, getMatId(nodeB.value)));
            }
        };

        if (chunk.isBorder)
        {
            uint8_t    axisIdx   = std::min<uint8_t>(chunk.borderAxis, 2);
            SVOAxis    axis      = static_cast<SVOAxis>(axisIdx);
            glm::ivec3 negOffset = chunk.minBound;
            glm::ivec3 posOffset = chunk.minBound;
            posOffset[axisIdx] += static_cast<int>(chunk.size);

            SVODualNode<T> rootA = SVODualTraversal::findSubtreeNode(svo, negOffset, chunk.size);
            SVODualNode<T> rootB = SVODualTraversal::findSubtreeNode(svo, posOffset, chunk.size);

            uint32_t cellSizeA = chunk.getCellSize();
            uint32_t cellSizeB = chunk.getSecondaryCellSize();

            auto isLeafA = [&](const SVODualNode<T>& node) -> bool { return node.size <= cellSizeA; };
            auto isLeafB = [&](const SVODualNode<T>& node) -> bool { return node.size <= cellSizeB; };

            SVODualTraversal::traverseFacePair(svo,
                                               rootA,
                                               rootB,
                                               axis,
                                               isLeafA,
                                               isLeafB,
                                               [&](const SVODualNode<T>& nodeA, const SVODualNode<T>& nodeB, SVOAxis ax)
                                               {
                                                   chunk.evaluatedCellCount++;
                                                   emitFaceQuads(nodeA, nodeB, ax, chunk.hasNegativeChunk, chunk.hasPositiveChunk);
                                               });
            return;
        }

        SVODualNode<T> rootNode = SVODualTraversal::findSubtreeNode(svo, chunk.minBound, chunk.size);
        if (rootNode.isVirtual || (rootNode.isLeaf && !isSolid(rootNode.value)))
        {
            return;
        }

        uint32_t cellSize = chunk.getCellSize();
        auto     isLeaf   = [&](const SVODualNode<T>& node) -> bool { return node.size <= cellSize; };
        auto     onNode   = [&](const SVODualNode<T>& leafNode) -> void
        {
            if (isSolid(leafNode.value))
            {
                chunk.evaluatedCellCount++;
            }
        };
        auto onFace = [&](const SVODualNode<T>& nodeA, const SVODualNode<T>& nodeB, SVOAxis ax) -> void { emitFaceQuads(nodeA, nodeB, ax, true, true); };

        SVODualTraversal::traverseSubtree(svo, rootNode, isLeaf, onNode, onFace);
    }

    template<typename T>
    void SVOCPUMesher::meshChunks(const SVO<T>& svo, std::vector<MeshChunk>& chunks)
    {
        generateBorderChunks(chunks);

#pragma omp parallel for schedule(dynamic)
        for (int i = 0; i < static_cast<int>(chunks.size()); i++)
        {
            meshChunk(svo, chunks[i]);
        }
    }

    template<typename T>
    std::unique_ptr<CADacomb::Triangulation> SVOCPUMesher::triangulate(const SVO<T>& svo, const SVOMesherOptions& options)
    {
        auto chunks = generateChunks(svo, options);
        meshChunks(svo, chunks);

        uint32_t masterDim      = svo.getDimension();
        double   voxelWorldSize = (masterDim > 0) ? (options.size.x / static_cast<double>(masterDim)) : 1.0;
        return extractTriangulation(chunks, options.origin, voxelWorldSize);
    }
}

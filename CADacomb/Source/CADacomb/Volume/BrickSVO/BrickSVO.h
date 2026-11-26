#pragma once

#include "Brick.h"
#include "BrickSVONode.h"
#include "CADacomb/Util/PagedVector.h"
#include "CADacomb/Volume/MergeStrategy/MeanMergeStrategy.h"
#include "CADacomb/Volume/MergeStrategy/MostPrevalentMergeStrategy.h"
#include "CADacomb/Volume/SVO/SVO.h"
#include "IBrickSVO.h"
#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <glm/glm.hpp>
#include <memory>
#include <mutex>
#include <string>
#include <typeinfo>
#include <unordered_set>
#include <vector>

#if defined(_OPENMP)
#include <omp.h>
#endif

namespace CADacomb
{
    template<typename T, unsigned short brickSize = 8, bool padded = false>
    class BrickSVO : public IBrickSVO
    {
      public:
        BrickSVO(uint32_t dimensionInput, const T& rootValue, uint32_t minLODDepth = 2);
        BrickSVO(uint32_t dimensionInput, const T& rootValue, const MergeStrategy<T>& strategy, uint32_t minLODDepth = 2);
        virtual ~BrickSVO();

        std::string getType() override;

        const MergeStrategy<T>&        getMergeStrategy() const;
        void                           setMergeStrategy(const MergeStrategy<T>& strategy);
        static const MergeStrategy<T>& defaultMergeStrategy();

        const PagedVector<BrickSVONode<T>>&                    getNodes() const;
        const PagedVector<Brick<T, brickSize, padded>, 10>&    getBricks() const;
        const std::vector<uint32_t>&                           getFreeNodeBlocks() const;
        uint32_t                                               getDimension() const;
        std::unordered_set<uint32_t>&                          getDirty();

        void cleanup(const glm::ivec3& start, const glm::ivec3& dimension);
        void set(const glm::ivec3& start, const glm::ivec3& dimension, const std::vector<T>& data);
        void setVoxel(const glm::ivec3& pos, const T& value);
        T    getVoxel(const glm::ivec3& pos) const;

        template<typename ClassifyBoxFn, typename EvalBlockFn>
        static std::unique_ptr<BrickSVO<T, brickSize, padded>> buildTopDown(uint32_t                dimensionInput,
                                                                            const T&                emptyValue,
                                                                            const MergeStrategy<T>& strategy,
                                                                            uint32_t                minLODDepthInput,
                                                                            ClassifyBoxFn&&         classifyBox,
                                                                            EvalBlockFn&&           evalBlock,
                                                                            const std::string&      progressMessage = "");

      private:
        bool     isBrickHomogenous(const uint32_t& brickPointer) const;
        void     deleteBrickAt(uint32_t brickIndex);
        void     freeChildBlock(uint32_t baseChildPtr);
        void     cleanupNode(uint32_t nodeIdx, glm::ivec3 nodePos, uint32_t nodeSize, const glm::ivec3& targetStart, const glm::ivec3& targetDim, uint32_t currentDepth);
        bool     intersectsAABB(const glm::ivec3& pos, uint32_t size, const glm::ivec3& start, const glm::ivec3& dim) const;
        void     processLeafCleanup(uint32_t nodeIdx);
        bool     tryCollapseChildren(uint32_t nodeIdx, uint32_t baseChildPtr);
        void     generateLODBrick(uint32_t nodeIdx, uint32_t baseChildPtr, glm::ivec3 nodePos, uint32_t childSize, const glm::ivec3& targetStart, const glm::ivec3& targetDim);
        uint32_t allocChildBlock(const T& initialValue);
        uint32_t allocBrick(uint32_t parentIdx, const T& fillValue);
        void     setNode(uint32_t nodeIdx, glm::ivec3 nodePos, uint32_t nodeSize, const glm::ivec3& targetStart, const glm::ivec3& targetDim, const glm::ivec3& inputDataDim, const std::vector<T>& data);
        void     updatePadding(uint32_t nodeIdx, glm::ivec3 nodePos, uint32_t nodeSize, const glm::ivec3& targetStart, const glm::ivec3& targetDim, uint32_t currentDepth);
        T        getVoxelAtDepth(const glm::ivec3& pos, uint32_t targetDepth, uint32_t currentDepth = 0, uint32_t nodeIdx = 0, glm::ivec3 nodePos = glm::ivec3(0), uint32_t nodeSize = 0) const;

        PagedVector<BrickSVONode<T>>                 nodes;
        PagedVector<Brick<T, brickSize, padded>, 10> bricks;
        std::unordered_set<uint32_t>                 dirtyBrickIndices;
        PagedVector<uint32_t, 10>                    brickParents;
        std::vector<uint32_t>                        freeNodeBlocks;
        uint32_t                                     dimension     = 128;
        uint32_t                                     minLODDepth   = 2;
        const MergeStrategy<T>*                      mergeStrategy = nullptr;
    };

    template<typename T, unsigned short brickSize, bool padded>
    const MergeStrategy<T>& BrickSVO<T, brickSize, padded>::defaultMergeStrategy()
    {
        if constexpr (std::is_same_v<T, bool>)
        {
            return MostPrevalentMergeStrategy<T>::instance();
        }
        else if constexpr (requires(T a) { a + a; })
        {
            return MeanMergeStrategy<T>::instance();
        }
        else
        {
            return MostPrevalentMergeStrategy<T>::instance();
        }
    }

    template<typename T, unsigned short brickSize, bool padded>
    BrickSVO<T, brickSize, padded>::BrickSVO(uint32_t dimensionInput, const T& rootValue, uint32_t minLODDepthInput)
      : BrickSVO(dimensionInput, rootValue, defaultMergeStrategy(), minLODDepthInput)
    {
    }

    template<typename T, unsigned short brickSize, bool padded>
    BrickSVO<T, brickSize, padded>::BrickSVO(uint32_t dimensionInput, const T& rootValue, const MergeStrategy<T>& strategy, uint32_t minLODDepthInput)
      : dimension(dimensionInput)
      , minLODDepth(minLODDepthInput)
      , mergeStrategy(&strategy)
    {
        assert(mergeStrategy != nullptr);
        static_assert((brickSize & (brickSize - 1)) == 0);
        assert(dimension > 0 && (dimension & (dimension - 1)) == 0);
        assert(dimension >= brickSize);
        nodes.emplace_back();
        nodes[0].value        = rootValue;
        nodes[0].childPointer = 0xFFFFFFFF;
        nodes[0].brickPointer = 0xFFFFFFFF;
    }

    template<typename T, unsigned short brickSize, bool padded>
    BrickSVO<T, brickSize, padded>::~BrickSVO()
    {
    }

    template<typename T, unsigned short brickSize, bool padded>
    std::string BrickSVO<T, brickSize, padded>::getType()
    {
        return typeid(T).name();
    }

    template<typename T, unsigned short brickSize, bool padded>
    const MergeStrategy<T>& BrickSVO<T, brickSize, padded>::getMergeStrategy() const
    {
        assert(mergeStrategy != nullptr);
        return *mergeStrategy;
    }

    template<typename T, unsigned short brickSize, bool padded>
    void BrickSVO<T, brickSize, padded>::setMergeStrategy(const MergeStrategy<T>& strategy)
    {
        mergeStrategy = &strategy;
    }

    template<typename T, unsigned short brickSize, bool padded>
    const PagedVector<BrickSVONode<T>>& BrickSVO<T, brickSize, padded>::getNodes() const
    {
        return nodes;
    }

    template<typename T, unsigned short brickSize, bool padded>
    const PagedVector<Brick<T, brickSize, padded>, 10>& BrickSVO<T, brickSize, padded>::getBricks() const
    {
        return bricks;
    }

    template<typename T, unsigned short brickSize, bool padded>
    const std::vector<uint32_t>& BrickSVO<T, brickSize, padded>::getFreeNodeBlocks() const
    {
        return freeNodeBlocks;
    }

    template<typename T, unsigned short brickSize, bool padded>
    uint32_t BrickSVO<T, brickSize, padded>::getDimension() const
    {
        return dimension;
    }

    template<typename T, unsigned short brickSize, bool padded>
    bool BrickSVO<T, brickSize, padded>::isBrickHomogenous(const uint32_t& brickPointer) const
    {
        const auto& brick = bricks[brickPointer].data;

        if constexpr (padded)
        {
            constexpr unsigned short allocatedSize  = brickSize + 2;
            constexpr size_t         firstCoreIndex = 1 + allocatedSize + (allocatedSize * allocatedSize);
            const auto&              firstVal       = brick[firstCoreIndex];

            for (unsigned short z = 1; z <= brickSize; ++z)
            {
                for (unsigned short y = 1; y <= brickSize; ++y)
                {
                    size_t rowStart = 1 + (y * allocatedSize) + (z * allocatedSize * allocatedSize);
                    size_t rowEnd   = rowStart + brickSize;

                    for (size_t i = rowStart; i < rowEnd; ++i)
                    {
                        if (firstVal != brick[i])
                            return false;
                    }
                }
            }
        }
        else
        {
            const auto&    firstVal = brick[0];
            constexpr auto brickDim = brickSize * brickSize * brickSize;
            for (size_t i = 1; i < brickDim; i++)
            {
                if (firstVal != brick[i])
                    return false;
            }
        }

        return true;
    }

    template<typename T, unsigned short brickSize, bool padded>
    void BrickSVO<T, brickSize, padded>::deleteBrickAt(uint32_t index)
    {
        if (index >= bricks.size())
            return;

        uint32_t lastIdx = static_cast<uint32_t>(bricks.size() - 1);
        if (index != lastIdx)
        {
            bricks[index]                           = std::move(bricks[lastIdx]);
            brickParents[index]                     = brickParents[lastIdx];
            nodes[brickParents[index]].brickPointer = index;
        }
        dirtyBrickIndices.insert(lastIdx);
        dirtyBrickIndices.insert(index);

        bricks.pop_back();
        brickParents.pop_back();
    }

    template<typename T, unsigned short brickSize, bool padded>
    void BrickSVO<T, brickSize, padded>::freeChildBlock(uint32_t baseChildPtr)
    {
        if (baseChildPtr == 0xFFFFFFFF)
        {
            return;
        }

        uint32_t lastBlockIndex = static_cast<uint32_t>(nodes.size() - 8);
        if (baseChildPtr == lastBlockIndex)
        {
            nodes.resize(lastBlockIndex);
        }
        else
        {
            freeNodeBlocks.push_back(baseChildPtr);
        }
    }

    template<typename T, unsigned short brickSize, bool padded>
    void BrickSVO<T, brickSize, padded>::cleanup(const glm::ivec3& start, const glm::ivec3& targetDimension)
    {
        cleanupNode(0, glm::ivec3(0), dimension, start, targetDimension, 0);
        if constexpr (padded)
        {
            updatePadding(0, glm::ivec3(0, 0, 0), dimension, start, targetDimension, 0);
        }
    }

    template<typename T, unsigned short brickSize, bool padded>
    void BrickSVO<T, brickSize, padded>::cleanupNode(uint32_t nodeIdx, glm::ivec3 nodePos, uint32_t nodeSize, const glm::ivec3& targetStart, const glm::ivec3& targetDim, uint32_t currentDepth)
    {
        if (!intersectsAABB(nodePos, nodeSize, targetStart, targetDim))
        {
            return;
        }

        if (nodeSize <= brickSize || nodes[nodeIdx].childPointer == 0xFFFFFFFF)
        {
            processLeafCleanup(nodeIdx);
            return;
        }

        uint32_t childSize    = nodeSize / 2;
        uint32_t baseChildPtr = nodes[nodeIdx].childPointer;

        for (int z = 0; z < 2; ++z)
        {
            for (int y = 0; y < 2; ++y)
            {
                for (int x = 0; x < 2; ++x)
                {
                    uint32_t   childIdx = baseChildPtr + (z * 4 + y * 2 + x);
                    glm::ivec3 childPos = nodePos + glm::ivec3(x, y, z) * (int)childSize;
                    cleanupNode(childIdx, childPos, childSize, targetStart, targetDim, currentDepth + 1);
                }
            }
        }

        if (tryCollapseChildren(nodeIdx, baseChildPtr))
        {
            return;
        }

        if (currentDepth < minLODDepth)
        {
            if (nodes[nodeIdx].brickPointer != 0xFFFFFFFF)
            {
                deleteBrickAt(nodes[nodeIdx].brickPointer);
                nodes[nodeIdx].brickPointer = 0xFFFFFFFF;
            }
            return;
        }

        generateLODBrick(nodeIdx, baseChildPtr, nodePos, childSize, targetStart, targetDim);
    }

    template<typename T, unsigned short brickSize, bool padded>
    bool BrickSVO<T, brickSize, padded>::intersectsAABB(const glm::ivec3& pos, uint32_t size, const glm::ivec3& start, const glm::ivec3& dim) const
    {
        return !(pos.x >= start.x + dim.x || pos.x + (int)size <= start.x || pos.y >= start.y + dim.y || pos.y + (int)size <= start.y || pos.z >= start.z + dim.z || pos.z + (int)size <= start.z);
    }

    template<typename T, unsigned short brickSize, bool padded>
    void BrickSVO<T, brickSize, padded>::processLeafCleanup(uint32_t nodeIdx)
    {
        if (nodes[nodeIdx].brickPointer != 0xFFFFFFFF && isBrickHomogenous(nodes[nodeIdx].brickPointer))
        {
            if constexpr (padded)
            {
                constexpr unsigned short allocatedSize  = brickSize + 2;
                constexpr size_t         firstCoreIndex = 1 + allocatedSize + (allocatedSize * allocatedSize);
                nodes[nodeIdx].value                    = bricks[nodes[nodeIdx].brickPointer].data[firstCoreIndex];
            }
            else
            {
                nodes[nodeIdx].value = bricks[nodes[nodeIdx].brickPointer].data[0];
            }
            deleteBrickAt(nodes[nodeIdx].brickPointer);
            nodes[nodeIdx].brickPointer = 0xFFFFFFFF;
        }
    }

    template<typename T, unsigned short brickSize, bool padded>
    bool BrickSVO<T, brickSize, padded>::tryCollapseChildren(uint32_t nodeIdx, uint32_t baseChildPtr)
    {
        T referenceValue = nodes[baseChildPtr].value;

        for (uint32_t i = 0; i < 8; ++i)
        {
            const auto& child = nodes[baseChildPtr + i];
            if (child.childPointer != 0xFFFFFFFF || child.brickPointer != 0xFFFFFFFF || !(child.value == referenceValue))
            {
                return false;
            }
        }

        nodes[nodeIdx].value = referenceValue;
        if (nodes[nodeIdx].brickPointer != 0xFFFFFFFF)
        {
            deleteBrickAt(nodes[nodeIdx].brickPointer);
            nodes[nodeIdx].brickPointer = 0xFFFFFFFF;
        }
        nodes[nodeIdx].childPointer = 0xFFFFFFFF;
        freeChildBlock(baseChildPtr);
        return true;
    }

    template<typename T, unsigned short brickSize, bool padded>
    void BrickSVO<T, brickSize, padded>::generateLODBrick(uint32_t nodeIdx, uint32_t baseChildPtr, glm::ivec3 nodePos, uint32_t childSize, const glm::ivec3& targetStart, const glm::ivec3& targetDim)
    {
        if (nodes[nodeIdx].brickPointer == 0xFFFFFFFF)
        {
            bricks.emplace_back();
            brickParents.emplace_back(nodeIdx);
            nodes[nodeIdx].brickPointer = static_cast<uint32_t>(bricks.size() - 1);
        }

        auto&                    lodBrick      = bricks[nodes[nodeIdx].brickPointer];
        constexpr unsigned short allocatedSize = padded ? (brickSize + 2) : brickSize;
        bool                     dirty         = false;

        for (int childZ = 0; childZ < 2; ++childZ)
        {
            for (int childY = 0; childY < 2; ++childY)
            {
                for (int childX = 0; childX < 2; ++childX)
                {
                    uint32_t   childIdx = baseChildPtr + (childZ * 4 + childY * 2 + childX);
                    glm::ivec3 childPos = nodePos + glm::ivec3(childX, childY, childZ) * (int)childSize;

                    if (!intersectsAABB(childPos, childSize, targetStart, targetDim))
                    {
                        continue;
                    }

                    const auto& childNode = nodes[childIdx];
                    int         zStart    = childZ * (brickSize / 2);
                    int         yStart    = childY * (brickSize / 2);
                    int         xStart    = childX * (brickSize / 2);

                    for (int bz = zStart; bz < zStart + brickSize / 2; ++bz)
                    {
                        for (int by = yStart; by < yStart + brickSize / 2; ++by)
                        {
                            for (int bx = xStart; bx < xStart + brickSize / 2; ++bx)
                            {
                                int localZ = bz % (brickSize / 2);
                                int localY = by % (brickSize / 2);
                                int localX = bx % (brickSize / 2);

                                std::array<T, 8> mergeInputs;

                                if (childNode.brickPointer != 0xFFFFFFFF)
                                {
                                    const auto& childBrick = bricks[childNode.brickPointer];
                                    int         mIdx       = 0;
                                    for (int sz = 0; sz < 2; sz++)
                                    {
                                        for (int sy = 0; sy < 2; sy++)
                                        {
                                            for (int sx = 0; sx < 2; sx++)
                                            {
                                                int srcX = localX * 2 + sx;
                                                int srcY = localY * 2 + sy;
                                                int srcZ = localZ * 2 + sz;

                                                int idx = padded ? (srcX + 1) + ((srcY + 1) * allocatedSize) + ((srcZ + 1) * allocatedSize * allocatedSize)
                                                                 : (srcZ * brickSize * brickSize) + (srcY * brickSize) + srcX;

                                                mergeInputs[mIdx++] = childBrick.data[idx];
                                            }
                                        }
                                    }
                                }
                                else
                                {
                                    mergeInputs.fill(childNode.value);
                                }

                                int lodIdx = padded ? (bx + 1) + ((by + 1) * allocatedSize) + ((bz + 1) * allocatedSize * allocatedSize) : (bz * brickSize * brickSize) + (by * brickSize) + bx;
                                assert(mergeStrategy != nullptr);
                                auto  merged = mergeStrategy->merge(mergeInputs);
                                auto& data   = lodBrick.data[lodIdx];

                                if (merged != data)
                                    dirty = true;

                                data = merged;
                            }
                        }
                    }
                }
            }
        }
        if (dirty)
            dirtyBrickIndices.insert(nodes[nodeIdx].brickPointer);
    }

    template<typename T, unsigned short brickSize, bool padded>
    void BrickSVO<T, brickSize, padded>::updatePadding(uint32_t nodeIdx, glm::ivec3 nodePos, uint32_t nodeSize, const glm::ivec3& targetStart, const glm::ivec3& targetDim, uint32_t currentDepth)
    {
        if (!intersectsAABB(nodePos, nodeSize, targetStart, targetDim))
        {
            return;
        }

        if constexpr (padded)
        {
            if (nodes[nodeIdx].brickPointer != 0xFFFFFFFF)
            {
                auto&                    lodBrick   = bricks[nodes[nodeIdx].brickPointer];
                constexpr unsigned short paddedSize = brickSize + 2;

                int  lodVoxelScale = (int)nodeSize / (int)brickSize;
                bool dirty         = false;
                for (int z = 0; z < paddedSize; ++z)
                {
                    for (int y = 0; y < paddedSize; ++y)
                    {
                        for (int x = 0; x < paddedSize; ++x)
                        {
                            bool isPadding = (x == 0 || x == brickSize + 1 || y == 0 || y == brickSize + 1 || z == 0 || z == brickSize + 1);
                            if (!isPadding)
                                continue;

                            glm::ivec3 samplePos = nodePos + glm::ivec3(x - 1, y - 1, z - 1) * lodVoxelScale;
                            samplePos            = glm::clamp(samplePos, glm::ivec3(0), glm::ivec3((int)dimension - 1));

                            size_t lodIdx  = x + (y * paddedSize) + (z * paddedSize * paddedSize);
                            auto&  data    = lodBrick.data[lodIdx];
                            auto   newData = getVoxelAtDepth(samplePos, currentDepth);
                            if (data != newData)
                                dirty = true;

                            data = newData;
                        }
                    }
                }
                if (dirty)
                    dirtyBrickIndices.insert(nodes[nodeIdx].brickPointer);
            }
        }

        if (nodeSize <= brickSize || nodes[nodeIdx].childPointer == 0xFFFFFFFF)
        {
            return;
        }

        uint32_t childSize    = nodeSize / 2;
        uint32_t baseChildPtr = nodes[nodeIdx].childPointer;

        for (int z = 0; z < 2; ++z)
        {
            for (int y = 0; y < 2; ++y)
            {
                for (int x = 0; x < 2; ++x)
                {
                    uint32_t   childIdx = baseChildPtr + (z * 4 + y * 2 + x);
                    glm::ivec3 childPos = nodePos + glm::ivec3(x, y, z) * (int)childSize;
                    updatePadding(childIdx, childPos, childSize, targetStart, targetDim, currentDepth + 1);
                }
            }
        }
    }

    template<typename T, unsigned short brickSize, bool padded>
    uint32_t BrickSVO<T, brickSize, padded>::allocChildBlock(const T& initialValue)
    {
        uint32_t newBlockStart = 0;
        if (!freeNodeBlocks.empty())
        {
            newBlockStart = freeNodeBlocks.back();
            freeNodeBlocks.pop_back();
        }
        else
        {
            newBlockStart = static_cast<uint32_t>(nodes.size());
            nodes.resize(newBlockStart + 8);
        }

        for (uint32_t i = 0; i < 8; ++i)
        {
            nodes[newBlockStart + i].value        = initialValue;
            nodes[newBlockStart + i].childPointer = 0xFFFFFFFF;
            nodes[newBlockStart + i].brickPointer = 0xFFFFFFFF;
        }

        return newBlockStart;
    }

    template<typename T, unsigned short brickSize, bool padded>
    uint32_t BrickSVO<T, brickSize, padded>::allocBrick(uint32_t parentIdx, const T& fillValue)
    {
        bricks.emplace_back();
        uint32_t newBrickIdx = static_cast<uint32_t>(bricks.size() - 1);

        std::fill(std::begin(bricks[newBrickIdx].data), std::end(bricks[newBrickIdx].data), fillValue);
        dirtyBrickIndices.insert(newBrickIdx);
        brickParents.push_back(parentIdx);
        return newBrickIdx;
    }

    template<typename T, unsigned short brickSize, bool padded>
    void BrickSVO<T, brickSize, padded>::set(const glm::ivec3& start, const glm::ivec3& dataDimension, const std::vector<T>& data)
    {
        setNode(0, glm::ivec3(0), dimension, start, dataDimension, dataDimension, data);
    }

    template<typename T, unsigned short brickSize, bool padded>
    void BrickSVO<T, brickSize, padded>::setNode(uint32_t              nodeIdx,
                                                 glm::ivec3            nodePos,
                                                 uint32_t              nodeSize,
                                                 const glm::ivec3&     targetStart,
                                                 const glm::ivec3&     targetDim,
                                                 const glm::ivec3&     inputDataDim,
                                                 const std::vector<T>& data)
    {
        if (nodePos.x >= targetStart.x + targetDim.x || nodePos.x + (int)nodeSize <= targetStart.x || nodePos.y >= targetStart.y + targetDim.y || nodePos.y + (int)nodeSize <= targetStart.y ||
            nodePos.z >= targetStart.z + targetDim.z || nodePos.z + (int)nodeSize <= targetStart.z)
        {
            return;
        }

        if (nodeSize == brickSize)
        {
            if (nodes[nodeIdx].brickPointer == 0xFFFFFFFF)
            {
                nodes[nodeIdx].brickPointer = allocBrick(nodeIdx, nodes[nodeIdx].value);
            }

            auto                     brickPointer  = nodes[nodeIdx].brickPointer;
            auto&                    brick         = bricks[brickPointer];
            constexpr unsigned short allocatedSize = padded ? (brickSize + 2) : brickSize;
            bool                     dirty         = false;
            for (int bz = 0; bz < brickSize; ++bz)
            {
                int worldZ = nodePos.z + bz;
                if (worldZ < targetStart.z || worldZ >= targetStart.z + targetDim.z)
                    continue;
                int inputZ = worldZ - targetStart.z;

                for (int by = 0; by < brickSize; ++by)
                {
                    int worldY = nodePos.y + by;
                    if (worldY < targetStart.y || worldY >= targetStart.y + targetDim.y)
                        continue;
                    int inputY = worldY - targetStart.y;

                    for (int bx = 0; bx < brickSize; ++bx)
                    {
                        int worldX = nodePos.x + bx;
                        if (worldX < targetStart.x || worldX >= targetStart.x + targetDim.x)
                            continue;
                        int inputX = worldX - targetStart.x;

                        size_t inputFlatIdx = ((size_t)inputZ * inputDataDim.y * inputDataDim.x) + ((size_t)inputY * inputDataDim.x) + inputX;

                        int brickFlatIdx;
                        if constexpr (padded)
                        {
                            brickFlatIdx = (bx + 1) + ((by + 1) * allocatedSize) + ((bz + 1) * allocatedSize * allocatedSize);
                        }
                        else
                        {
                            brickFlatIdx = (bz * brickSize * brickSize) + (by * brickSize) + bx;
                        }
                        if (data[inputFlatIdx] != brick.data[brickFlatIdx])
                            dirty = true;
                        brick.data[brickFlatIdx] = data[inputFlatIdx];
                    }
                }
            }
            if (dirty)
                dirtyBrickIndices.insert(brickPointer);
            return;
        }

        if (nodes[nodeIdx].childPointer == 0xFFFFFFFF)
        {
            nodes[nodeIdx].childPointer = allocChildBlock(nodes[nodeIdx].value);
        }

        uint32_t childSize    = nodeSize / 2;
        uint32_t baseChildPtr = nodes[nodeIdx].childPointer;

        for (int childZ = 0; childZ < 2; ++childZ)
        {
            for (int childY = 0; childY < 2; ++childY)
            {
                for (int childX = 0; childX < 2; ++childX)
                {
                    uint32_t   childIdx = baseChildPtr + (childZ * 4 + childY * 2 + childX);
                    glm::ivec3 childPos = nodePos + glm::ivec3(childX, childY, childZ) * (int)childSize;

                    setNode(childIdx, childPos, childSize, targetStart, targetDim, inputDataDim, data);
                }
            }
        }
    }

    template<typename T, unsigned short brickSize, bool padded>
    void BrickSVO<T, brickSize, padded>::setVoxel(const glm::ivec3& pos, const T& value)
    {
        if (pos.x < 0 || pos.x >= static_cast<int>(dimension) || pos.y < 0 || pos.y >= static_cast<int>(dimension) || pos.z < 0 || pos.z >= static_cast<int>(dimension))
        {
            return;
        }

        uint32_t   nodeIdx = 0;
        glm::ivec3 nodePos(0, 0, 0);
        uint32_t   nodeSize = dimension;

        while (nodeSize > brickSize)
        {
            if (nodes[nodeIdx].childPointer == 0xFFFFFFFF)
            {
                if (nodes[nodeIdx].value == value && nodes[nodeIdx].brickPointer == 0xFFFFFFFF)
                {
                    return;
                }
                nodes[nodeIdx].childPointer = allocChildBlock(nodes[nodeIdx].value);
            }

            uint32_t childSize = nodeSize >> 1;
            int      childX    = (pos.x >= nodePos.x + static_cast<int>(childSize)) ? 1 : 0;
            int      childY    = (pos.y >= nodePos.y + static_cast<int>(childSize)) ? 1 : 0;
            int      childZ    = (pos.z >= nodePos.z + static_cast<int>(childSize)) ? 1 : 0;

            nodePos.x += childX * static_cast<int>(childSize);
            nodePos.y += childY * static_cast<int>(childSize);
            nodePos.z += childZ * static_cast<int>(childSize);

            nodeIdx  = nodes[nodeIdx].childPointer + static_cast<uint32_t>(childZ * 4 + childY * 2 + childX);
            nodeSize = childSize;
        }

        if (nodes[nodeIdx].brickPointer == 0xFFFFFFFF)
        {
            if (nodes[nodeIdx].value == value)
            {
                return;
            }
            nodes[nodeIdx].brickPointer = allocBrick(nodeIdx, nodes[nodeIdx].value);
        }

        uint32_t brickPointer = nodes[nodeIdx].brickPointer;
        auto&    brick        = bricks[brickPointer];

        int bx = pos.x - nodePos.x;
        int by = pos.y - nodePos.y;
        int bz = pos.z - nodePos.z;

        size_t brickFlatIdx = 0;
        if constexpr (padded)
        {
            constexpr unsigned short allocatedSize = brickSize + 2;
            brickFlatIdx                           = static_cast<size_t>((bx + 1) + ((by + 1) * allocatedSize) + ((bz + 1) * allocatedSize * allocatedSize));
        }
        else
        {
            brickFlatIdx = static_cast<size_t>((bz * brickSize * brickSize) + (by * brickSize) + bx);
        }

        if (brick.data[brickFlatIdx] != value)
        {
            brick.data[brickFlatIdx] = value;
            dirtyBrickIndices.insert(brickPointer);
        }
    }

    template<typename T, unsigned short brickSize, bool padded>
    T BrickSVO<T, brickSize, padded>::getVoxel(const glm::ivec3& pos) const
    {
        if (pos.x < 0 || pos.x >= (int)dimension || pos.y < 0 || pos.y >= (int)dimension || pos.z < 0 || pos.z >= (int)dimension)
        {
            return T{};
        }

        uint32_t   nodeIdx = 0;
        glm::ivec3 nodePos(0, 0, 0);
        uint32_t   nodeSize = dimension;

        while (nodeSize > brickSize && nodes[nodeIdx].childPointer != 0xFFFFFFFF)
        {
            uint32_t childSize    = nodeSize / 2;
            uint32_t baseChildPtr = nodes[nodeIdx].childPointer;

            int childX = (pos.x >= nodePos.x + (int)childSize) ? 1 : 0;
            int childY = (pos.y >= nodePos.y + (int)childSize) ? 1 : 0;
            int childZ = (pos.z >= nodePos.z + (int)childSize) ? 1 : 0;

            nodePos.x += childX * (int)childSize;
            nodePos.y += childY * (int)childSize;
            nodePos.z += childZ * (int)childSize;

            nodeIdx  = baseChildPtr + (childZ * 4 + childY * 2 + childX);
            nodeSize = childSize;
        }

        if (nodes[nodeIdx].brickPointer != 0xFFFFFFFF)
        {
            const auto& brick = bricks[nodes[nodeIdx].brickPointer];

            int bx = pos.x - nodePos.x;
            int by = pos.y - nodePos.y;
            int bz = pos.z - nodePos.z;

            int brickFlatIdx;
            if constexpr (padded)
            {
                constexpr unsigned short allocatedSize = brickSize + 2;
                brickFlatIdx                           = (bx + 1) + ((by + 1) * allocatedSize) + ((bz + 1) * allocatedSize * allocatedSize);
            }
            else
            {
                brickFlatIdx = (bz * brickSize * brickSize) + (by * brickSize) + bx;
            }

            return brick.data[brickFlatIdx];
        }

        return nodes[nodeIdx].value;
    }

    template<typename T, unsigned short brickSize, bool padded>
    T BrickSVO<T, brickSize, padded>::getVoxelAtDepth(const glm::ivec3& pos, uint32_t targetDepth, uint32_t currentDepth, uint32_t nodeIdx, glm::ivec3 nodePos, uint32_t nodeSize) const
    {
        if (nodeSize == 0)
        {
            nodeSize = dimension;
        }

        if (nodes[nodeIdx].childPointer == 0xFFFFFFFF || currentDepth == targetDepth)
        {
            if (nodes[nodeIdx].brickPointer != 0xFFFFFFFF)
            {
                const auto& brick = bricks[nodes[nodeIdx].brickPointer];

                int lodVoxelScale = (int)nodeSize / (int)brickSize;
                int bx            = (pos.x - nodePos.x) / lodVoxelScale;
                int by            = (pos.y - nodePos.y) / lodVoxelScale;
                int bz            = (pos.z - nodePos.z) / lodVoxelScale;

                bx = glm::clamp(bx, 0, (int)brickSize - 1);
                by = glm::clamp(by, 0, (int)brickSize - 1);
                bz = glm::clamp(bz, 0, (int)brickSize - 1);

                size_t idx = padded ? (bx + 1) + ((by + 1) * (brickSize + 2)) + ((bz + 1) * (brickSize + 2) * (brickSize + 2)) : (bz * brickSize * brickSize) + (by * brickSize) + bx;

                return brick.data[idx];
            }
            return nodes[nodeIdx].value;
        }

        uint32_t childSize = nodeSize / 2;
        int      cx        = (pos.x - nodePos.x) / (int)childSize;
        int      cy        = (pos.y - nodePos.y) / (int)childSize;
        int      cz        = (pos.z - nodePos.z) / (int)childSize;

        cx = glm::clamp(cx, 0, 1);
        cy = glm::clamp(cy, 0, 1);
        cz = glm::clamp(cz, 0, 1);

        uint32_t   childIdx = nodes[nodeIdx].childPointer + (cz * 4 + cy * 2 + cx);
        glm::ivec3 childPos = nodePos + glm::ivec3(cx, cy, cz) * (int)childSize;

        return getVoxelAtDepth(pos, targetDepth, currentDepth + 1, childIdx, childPos, childSize);
    }

    template<typename T, unsigned short brickSize, bool padded>
    std::unordered_set<uint32_t>& BrickSVO<T, brickSize, padded>::getDirty()
    {
        return dirtyBrickIndices;
    }

    template<typename T, unsigned short brickSize, bool padded>
    template<typename ClassifyBoxFn, typename EvalBlockFn>
    std::unique_ptr<BrickSVO<T, brickSize, padded>> BrickSVO<T, brickSize, padded>::buildTopDown(uint32_t                dimensionInput,
                                                                                                 const T&                emptyValue,
                                                                                                 const MergeStrategy<T>& strategy,
                                                                                                 uint32_t                minLODDepthInput,
                                                                                                 ClassifyBoxFn&&         classifyBox,
                                                                                                 EvalBlockFn&&           evalBlock,
                                                                                                 const std::string&      progressMessage)
    {
        assert(dimensionInput >= brickSize && (dimensionInput & (dimensionInput - 1)) == 0);

        auto svo = std::make_unique<BrickSVO<T, brickSize, padded>>(dimensionInput, emptyValue, strategy, minLODDepthInput);

        uint32_t frontierDim = brickSize;
        if (dimensionInput >= static_cast<uint32_t>(brickSize) * 16u)
        {
            frontierDim = std::max(static_cast<uint32_t>(brickSize), dimensionInput >> 4);
        }
        else if (dimensionInput >= static_cast<uint32_t>(brickSize) * 4u)
        {
            frontierDim = std::max(static_cast<uint32_t>(brickSize), dimensionInput >> 2);
        }
        else
        {
            frontierDim = dimensionInput;
        }

        std::vector<uint32_t>   taskNodeIndices;
        std::vector<glm::ivec3> taskOffsets;
        std::vector<uint32_t>   taskDims;
        std::vector<uint32_t>   taskDepths;

        auto collectFrontier = [&](auto& self, uint32_t nodeIdx, const glm::ivec3& offset, uint32_t curDim, uint32_t depth) -> void
        {
            T uniformVal = emptyValue;
            if (classifyBox(offset, curDim, uniformVal))
            {
                svo->nodes[nodeIdx].value        = uniformVal;
                svo->nodes[nodeIdx].childPointer = 0xFFFFFFFF;
                svo->nodes[nodeIdx].brickPointer = 0xFFFFFFFF;
                return;
            }

            if (curDim <= frontierDim)
            {
                taskNodeIndices.push_back(nodeIdx);
                taskOffsets.push_back(offset);
                taskDims.push_back(curDim);
                taskDepths.push_back(depth);
                return;
            }

            uint32_t childBlock              = svo->allocChildBlock(emptyValue);
            svo->nodes[nodeIdx].childPointer = childBlock;
            uint32_t half                    = curDim >> 1;

            for (int cz = 0; cz < 2; cz++)
            {
                for (int cy = 0; cy < 2; cy++)
                {
                    for (int cx = 0; cx < 2; cx++)
                    {
                        uint32_t   childIdx = childBlock + static_cast<uint32_t>(cz * 4 + cy * 2 + cx);
                        glm::ivec3 childOff = offset + glm::ivec3(cx, cy, cz) * static_cast<int>(half);
                        self(self, childIdx, childOff, half, depth + 1);
                    }
                }
            }
        };

        collectFrontier(collectFrontier, 0, glm::ivec3(0), dimensionInput, 0);

        if (!taskNodeIndices.empty())
        {
            constexpr size_t maxNodePages  = size_t(1) << (32 - 12);
            constexpr size_t maxBrickPages = size_t(1) << (32 - 10);
            size_t           nodePages     = maxNodePages;
            size_t           brickPages    = maxBrickPages;
            if (dimensionInput <= 512)
            {
                size_t bricksPerSide = dimensionInput / brickSize;
                size_t maxBricks     = bricksPerSide * bricksPerSide * bricksPerSide * 2 + 64;
                nodePages            = std::min(maxNodePages, (maxBricks * 2 + 4095) / 4096 + 16);
                brickPages           = std::min(maxBrickPages, (maxBricks + 1023) / 1024 + 16);
            }
            svo->nodes.reservePageTable(nodePages);
            svo->bricks.reservePageTable(brickPages);
            svo->brickParents.reservePageTable(brickPages);

            std::unique_ptr<ProgressBar> progressBar;
            if (!progressMessage.empty())
            {
                progressBar = std::make_unique<ProgressBar>(taskNodeIndices.size(), progressMessage);
            }

            std::mutex         nodeAllocMutex;
            std::mutex         brickAllocMutex;
            std::atomic<bool>  canceled{ false };
            constexpr uint32_t threadChunkNodes = 512;
            constexpr size_t   brickVoxelCount  = static_cast<size_t>(brickSize) * brickSize * brickSize;

#pragma omp parallel shared(svo, taskNodeIndices, taskOffsets, taskDims, taskDepths, nodeAllocMutex, brickAllocMutex, progressBar, canceled)
            {
                uint32_t              chunkCursor = 0;
                uint32_t              chunkEnd    = 0;
                std::vector<uint32_t> localFreeBlocks;
                std::vector<T>        localBrickBuffer(brickVoxelCount);

                auto allocBlock = [&]() -> uint32_t
                {
                    if (!localFreeBlocks.empty())
                    {
                        uint32_t ptr = localFreeBlocks.back();
                        localFreeBlocks.pop_back();
                        return ptr;
                    }

                    if (chunkCursor + 8 > chunkEnd)
                    {
                        std::lock_guard<std::mutex> lock(nodeAllocMutex);
                        size_t                      start = svo->nodes.size();
                        svo->nodes.resize(start + threadChunkNodes);
                        chunkCursor = static_cast<uint32_t>(start);
                        chunkEnd    = chunkCursor + threadChunkNodes;
                    }

                    uint32_t ptr = chunkCursor;
                    chunkCursor += 8;
                    return ptr;
                };

                auto freeBlock = [&](uint32_t ptr) -> void
                {
                    if (ptr + 8 == chunkCursor)
                    {
                        chunkCursor -= 8;
                    }
                    else
                    {
                        localFreeBlocks.push_back(ptr);
                    }
                };

                auto allocFilledBrick = [&](uint32_t parentIdx, const std::vector<T>& voxelData) -> uint32_t
                {
                    uint32_t brickIdx = 0;
                    {
                        std::lock_guard<std::mutex> lock(brickAllocMutex);
                        svo->bricks.emplace_back();
                        brickIdx = static_cast<uint32_t>(svo->bricks.size() - 1);
                        svo->brickParents.push_back(parentIdx);
                    }

                    auto&                    brick         = svo->bricks[brickIdx];
                    constexpr unsigned short allocatedSize = padded ? (brickSize + 2) : brickSize;

                    if constexpr (padded)
                    {
                        std::fill(std::begin(brick.data), std::end(brick.data), emptyValue);
                        for (int bz = 0; bz < brickSize; bz++)
                        {
                            for (int by = 0; by < brickSize; by++)
                            {
                                for (int bx = 0; bx < brickSize; bx++)
                                {
                                    size_t srcIdx      = static_cast<size_t>(bx + by * brickSize + bz * brickSize * brickSize);
                                    size_t dstIdx      = static_cast<size_t>((bx + 1) + (by + 1) * allocatedSize + (bz + 1) * allocatedSize * allocatedSize);
                                    brick.data[dstIdx] = voxelData[srcIdx];
                                }
                            }
                        }
                    }
                    else
                    {
                        for (size_t i = 0; i < brickVoxelCount; i++)
                        {
                            brick.data[i] = voxelData[i];
                        }
                    }

                    return brickIdx;
                };

                auto buildSubtree = [&](auto& self, uint32_t nodeIdx, const glm::ivec3& offset, uint32_t curDim) -> bool
                {
                    T uniformVal = emptyValue;
                    if (classifyBox(offset, curDim, uniformVal))
                    {
                        svo->nodes[nodeIdx].value        = uniformVal;
                        svo->nodes[nodeIdx].childPointer = 0xFFFFFFFF;
                        svo->nodes[nodeIdx].brickPointer = 0xFFFFFFFF;
                        return true;
                    }

                    if (curDim == brickSize)
                    {
                        evalBlock(offset, brickSize, localBrickBuffer.data());

                        T    v0      = localBrickBuffer[0];
                        bool allSame = true;
                        for (size_t i = 1; i < brickVoxelCount; i++)
                        {
                            if (localBrickBuffer[i] != v0)
                            {
                                allSame = false;
                                break;
                            }
                        }

                        if (allSame)
                        {
                            svo->nodes[nodeIdx].value        = v0;
                            svo->nodes[nodeIdx].childPointer = 0xFFFFFFFF;
                            svo->nodes[nodeIdx].brickPointer = 0xFFFFFFFF;
                            return true;
                        }

                        uint32_t brickIdx                = allocFilledBrick(nodeIdx, localBrickBuffer);
                        svo->nodes[nodeIdx].value        = v0;
                        svo->nodes[nodeIdx].childPointer = 0xFFFFFFFF;
                        svo->nodes[nodeIdx].brickPointer = brickIdx;
                        return false;
                    }

                    uint32_t childPtr                = allocBlock();
                    svo->nodes[nodeIdx].childPointer = childPtr;
                    svo->nodes[nodeIdx].brickPointer = 0xFFFFFFFF;
                    uint32_t half                    = curDim >> 1;

                    bool allChildrenUniform = true;
                    for (int cz = 0; cz < 2; cz++)
                    {
                        for (int cy = 0; cy < 2; cy++)
                        {
                            for (int cx = 0; cx < 2; cx++)
                            {
                                uint32_t   childIdx = childPtr + static_cast<uint32_t>(cz * 4 + cy * 2 + cx);
                                glm::ivec3 childOff = offset + glm::ivec3(cx, cy, cz) * static_cast<int>(half);
                                if (!self(self, childIdx, childOff, half))
                                {
                                    allChildrenUniform = false;
                                }
                            }
                        }
                    }

                    if (allChildrenUniform)
                    {
                        T    v0      = svo->nodes[childPtr].value;
                        bool allSame = true;
                        for (uint32_t i = 1; i < 8; i++)
                        {
                            if (svo->nodes[childPtr + i].value != v0)
                            {
                                allSame = false;
                                break;
                            }
                        }

                        if (allSame)
                        {
                            svo->nodes[nodeIdx].value        = v0;
                            svo->nodes[nodeIdx].childPointer = 0xFFFFFFFF;
                            svo->nodes[nodeIdx].brickPointer = 0xFFFFFFFF;
                            freeBlock(childPtr);
                            return true;
                        }
                    }

                    std::array<T, 8> childVals;
                    for (uint32_t i = 0; i < 8; i++)
                    {
                        childVals[i] = svo->nodes[childPtr + i].value;
                    }
                    svo->nodes[nodeIdx].value = strategy.merge(childVals);
                    return false;
                };

#pragma omp for schedule(dynamic)
                for (int t = 0; t < static_cast<int>(taskNodeIndices.size()); t++)
                {
                    if (progressBar && progressBar->isCanceled())
                    {
                        canceled.store(true, std::memory_order_relaxed);
                        continue;
                    }

                    buildSubtree(buildSubtree, taskNodeIndices[static_cast<size_t>(t)], taskOffsets[static_cast<size_t>(t)], taskDims[static_cast<size_t>(t)]);

                    if (progressBar)
                    {
                        progressBar->addProgress(1);
                    }
                }

                if (!localFreeBlocks.empty() || chunkCursor < chunkEnd)
                {
                    std::lock_guard<std::mutex> lock(nodeAllocMutex);
                    for (uint32_t ptr : localFreeBlocks)
                    {
                        svo->freeNodeBlocks.push_back(ptr);
                    }
                    while (chunkCursor + 8 <= chunkEnd)
                    {
                        svo->freeNodeBlocks.push_back(chunkCursor);
                        chunkCursor += 8;
                    }
                }
            }

            if (canceled.load(std::memory_order_relaxed) || (progressBar && progressBar->isCanceled()))
            {
                return nullptr;
            }
        }

        svo->cleanup(glm::ivec3(0), glm::ivec3(static_cast<int>(dimensionInput)));

        if (svo->nodes[0].childPointer == 0xFFFFFFFF && svo->nodes[0].brickPointer == 0xFFFFFFFF)
        {
            svo->nodes.resize(1);
            svo->freeNodeBlocks.clear();
        }

        svo->nodes.shrinkToFit();
        svo->bricks.shrinkToFit();
        svo->brickParents.shrinkToFit();
        return svo;
    }
}

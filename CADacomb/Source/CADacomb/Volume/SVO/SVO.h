#pragma once

#include <algorithm>
#include <array>
#include <cassert>
#include <concepts>
#include <functional>
#include <glm/glm.hpp>
#include <memory>
#include <optional>
#include <queue>
#include <string>
#include <type_traits>
#include <vector>

#include "AnySVO.h"
#include "CADacomb/Volume/MergeStrategy/MeanMergeStrategy.h"
#include "CADacomb/Volume/MergeStrategy/MergeStrategy.h"
#include "CADacomb/Volume/MergeStrategy/MostPrevalentMergeStrategy.h"
#include "SVONode.h"

namespace CADacomb
{
    template<typename T>
    class SVO : public AnySVO
    {
      public:
        static const MergeStrategy<T>& defaultMergeStrategy();

        SVO(uint32_t dimensionInput, const T& rootValue);
        SVO(uint32_t dimensionInput, const T& rootValue, const MergeStrategy<T>& strategy);
        SVO(const std::vector<T>& grid, uint32_t dimensionInput);
        SVO(const std::vector<T>& grid, uint32_t dimensionInput, const MergeStrategy<T>& strategy);
        SVO(uint32_t dimensionInput, std::vector<SVONode<T>> nodesInput, std::vector<uint32_t> parentsInput, std::vector<uint32_t> freeBlocksInput, const MergeStrategy<T>& strategy);
        virtual ~SVO();

        std::string getType() override;

        void setVoxel(const glm::ivec3& target, T value);
        T    getVoxel(const glm::ivec3& target, int maxDepth = -1) const;
        void get3DArray(std::vector<T>& outBuffer, const glm::ivec3& start, const glm::ivec3& size, int depth = -1) const;
        bool isHomogenous(const glm::ivec3& start, const glm::ivec3& size) const;

        std::unique_ptr<SVO<T>> getSubSVO(const glm::ivec3& offset, uint32_t depth) const;
        void                    setSubSVO(const glm::ivec3& offset, const SVO<T>& subSVO);
        void                    setSubSVO(const glm::ivec3& offset, const std::unique_ptr<SVO<T>>& subSVO);

        const MergeStrategy<T>& getMergeStrategy() const;
        void                    setMergeStrategy(const MergeStrategy<T>& strategy);

        const std::vector<SVONode<T>>& getRaw() const;
        const std::vector<uint32_t>&   getParents() const;
        const std::vector<uint32_t>&   getFreeBlocks() const;
        uint32_t                       getDimension() const;

        template<typename Visitor>
        void traverse(Visitor&& visitor) const;

        template<typename LeafVisitor>
        void traverseLeaves(LeafVisitor&& leafVisitor) const;

      private:
        uint32_t   addNodeBlock(uint32_t parentIndex);
        void       removeNodeBlock(uint32_t address);
        void       deleteSubtree(uint32_t nodeIndex);
        short      pickChild(const glm::ivec3& target, const glm::ivec3& currentOffset, uint32_t currentDimension) const;
        glm::ivec3 nextOffset(short childnode, const glm::ivec3& currentOffset, uint32_t currentDimension) const;
        void       setVoxel(const glm::ivec3& target, const T& value, uint32_t currentNode, const glm::ivec3& currentOffset, uint32_t currentDimension);
        T          getVoxel(const glm::ivec3& target, uint32_t currentNode, const glm::ivec3& currentOffset, uint32_t currentDimension, int maxDepth, int depth) const;
        T          average(const std::array<T, 8>& inputs);
        bool       buildFromGrid(const std::vector<T>& grid, uint32_t currentNode, const glm::ivec3& currentOffset, uint32_t currentDimension);

        uint32_t                dimension = 128;
        std::vector<SVONode<T>> nodes;
        std::vector<uint32_t>   parents;
        std::vector<uint32_t>   freeBlocks;
        const MergeStrategy<T>* mergeStrategy = nullptr;
    };

    template<typename T>
    const MergeStrategy<T>& SVO<T>::defaultMergeStrategy()
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

    template<typename T>
    SVO<T>::SVO(uint32_t dimensionInput, const T& rootValue)
      : SVO(dimensionInput, rootValue, defaultMergeStrategy())
    {
    }

    template<typename T>
    SVO<T>::SVO(uint32_t dimensionInput, const T& rootValue, const MergeStrategy<T>& strategy)
      : dimension(dimensionInput)
      , mergeStrategy(&strategy)
    {
        assert(dimension > 0 && (dimension & (dimension - 1)) == 0);
        assert(mergeStrategy != nullptr);
        nodes.reserve(8192);
        parents.reserve(8192);
        nodes.emplace_back();
        parents.emplace_back(0);
        nodes[0].value        = rootValue;
        nodes[0].childPointer = 0xFFFFFFFF;
    }

    template<typename T>
    SVO<T>::SVO(const std::vector<T>& grid, uint32_t dimensionInput)
      : SVO(grid, dimensionInput, defaultMergeStrategy())
    {
    }

    template<typename T>
    SVO<T>::SVO(const std::vector<T>& grid, uint32_t dimensionInput, const MergeStrategy<T>& strategy)
      : dimension(dimensionInput)
      , mergeStrategy(&strategy)
    {
        assert(dimension > 0 && (dimension & (dimension - 1)) == 0);
        assert(grid.size() == static_cast<size_t>(dimension) * dimension * dimension);
        assert(mergeStrategy != nullptr);

        nodes.reserve(8192);
        parents.reserve(8192);
        nodes.emplace_back();
        parents.emplace_back(0);
        nodes[0].value        = T{};
        nodes[0].childPointer = 0xFFFFFFFF;

        if (dimension == 1)
        {
            nodes[0].value = grid.empty() ? T{} : grid[0];
            return;
        }

        buildFromGrid(grid, 0, glm::ivec3(0), dimension);
    }

    template<typename T>
    SVO<T>::SVO(uint32_t dimensionInput, std::vector<SVONode<T>> nodesInput, std::vector<uint32_t> parentsInput, std::vector<uint32_t> freeBlocksInput, const MergeStrategy<T>& strategy)
      : dimension(dimensionInput)
      , nodes(std::move(nodesInput))
      , parents(std::move(parentsInput))
      , freeBlocks(std::move(freeBlocksInput))
      , mergeStrategy(&strategy)
    {
        assert(dimension > 0 && (dimension & (dimension - 1)) == 0);
        assert(mergeStrategy != nullptr);
    }

    template<typename T>
    SVO<T>::~SVO()
    {
    }

    template<typename T>
    const std::vector<SVONode<T>>& SVO<T>::getRaw() const
    {
        return nodes;
    }

    template<typename T>
    const std::vector<uint32_t>& SVO<T>::getParents() const
    {
        return parents;
    }

    template<typename T>
    const std::vector<uint32_t>& SVO<T>::getFreeBlocks() const
    {
        return freeBlocks;
    }

    template<typename T>
    uint32_t SVO<T>::getDimension() const
    {
        return dimension;
    }

    template<typename T>
    std::string SVO<T>::getType()
    {
        return typeid(T).name();
    }

    template<typename T>
    const MergeStrategy<T>& SVO<T>::getMergeStrategy() const
    {
        assert(mergeStrategy != nullptr);
        return *mergeStrategy;
    }

    template<typename T>
    void SVO<T>::setMergeStrategy(const MergeStrategy<T>& strategy)
    {
        mergeStrategy = &strategy;
    }

    template<typename T>
    uint32_t SVO<T>::addNodeBlock(uint32_t parentIndex)
    {
        uint32_t result;
        T        value = nodes[parentIndex].value;

        if (!freeBlocks.empty())
        {
            result = freeBlocks.back();
            freeBlocks.pop_back();
            for (size_t i = 0; i < 8; i++)
            {
                nodes[result + i].value        = value;
                nodes[result + i].childPointer = 0xFFFFFFFF;
                parents[result + i]            = parentIndex;
            }
            return result;
        }

        result = static_cast<uint32_t>(nodes.size());

        if (result + 8 > nodes.capacity())
        {
            size_t newCapacity = nodes.capacity() + 8192;
            nodes.reserve(newCapacity);
            parents.reserve(newCapacity);
        }

        parents.resize(result + 8);
        nodes.resize(result + 8);
        for (size_t i = 0; i < 8; i++)
        {
            nodes[result + i].value        = value;
            nodes[result + i].childPointer = 0xFFFFFFFF;
            parents[result + i]            = parentIndex;
        }
        return result;
    }

    template<typename T>
    void SVO<T>::removeNodeBlock(uint32_t address)
    {
        auto&    current  = nodes[address];
        uint32_t childPtr = current.childPointer;
        assert(childPtr != 0xFFFFFFFF);

        for (uint32_t i = 0; i < 8; i++)
        {
            assert(nodes[childPtr + i].childPointer == 0xFFFFFFFF);
        }
        current.childPointer = 0xFFFFFFFF;

        uint32_t lastBlockIndex = static_cast<uint32_t>(nodes.size() - 8);
        if (childPtr == lastBlockIndex)
        {
            nodes.resize(lastBlockIndex);
            parents.resize(lastBlockIndex);
        }
        else
        {
            freeBlocks.push_back(childPtr);
        }
    }

    template<typename T>
    short SVO<T>::pickChild(const glm::ivec3& target, const glm::ivec3& currentOffset, uint32_t currentDimension) const
    {
        uint32_t half       = currentDimension / 2;
        short    childIndex = 0;
        if (target.x >= currentOffset.x + static_cast<int>(half))
        {
            childIndex |= 1;
        }
        if (target.y >= currentOffset.y + static_cast<int>(half))
        {
            childIndex |= 2;
        }
        if (target.z >= currentOffset.z + static_cast<int>(half))
        {
            childIndex |= 4;
        }
        return childIndex;
    }

    template<typename T>
    glm::ivec3 SVO<T>::nextOffset(short childIndex, const glm::ivec3& currentOffset, uint32_t currentDimension) const
    {
        uint32_t   half       = currentDimension / 2;
        glm::ivec3 nextOffset = currentOffset;
        if (childIndex & 1)
        {
            nextOffset.x += half;
        }
        if (childIndex & 2)
        {
            nextOffset.y += half;
        }
        if (childIndex & 4)
        {
            nextOffset.z += half;
        }
        return nextOffset;
    }

    template<typename T>
    void SVO<T>::setVoxel(const glm::ivec3& target, const T& value, uint32_t currentNode, const glm::ivec3& currentOffset, uint32_t currentDimension)
    {
        if (currentDimension == 1)
        {
            nodes[currentNode].value = value;
            return;
        }

        if (nodes[currentNode].childPointer == 0xFFFFFFFF)
        {
            if (nodes[currentNode].value == value)
            {
                return;
            }
            uint32_t newBlock               = addNodeBlock(currentNode);
            nodes[currentNode].childPointer = newBlock;
        }
        auto childIndex    = pickChild(target, currentOffset, currentDimension);
        auto offset        = nextOffset(childIndex, currentOffset, currentDimension);
        auto nextDimension = currentDimension / 2;
        auto nextNodeIndex = nodes[currentNode].childPointer + childIndex;

        setVoxel(target, value, nextNodeIndex, offset, nextDimension);

        auto childPointer = nodes[currentNode].childPointer;
        T    val          = nodes[childPointer].value;
        for (size_t i = 0; i < 8; i++)
        {
            if (val != nodes[childPointer + i].value || nodes[childPointer + i].childPointer != 0xFFFFFFFF)
            {
                std::array<T, 8> toAverage;
                for (size_t av = 0; av < 8; av++)
                {
                    toAverage[av] = nodes[childPointer + av].value;
                }
                nodes[currentNode].value = average(toAverage);
                return;
            }
        }
        nodes[currentNode].value = val;
        removeNodeBlock(currentNode);
    }

    template<typename T>
    T SVO<T>::getVoxel(const glm::ivec3& target, int maxDepth) const
    {
        assert(maxDepth >= -1);
        assert(target[0] >= 0 && target[1] >= 0 && target[2] >= 0);
        assert(target[0] < static_cast<int>(dimension) && target[1] < static_cast<int>(dimension) && target[2] < static_cast<int>(dimension));
        return getVoxel(target, 0, glm::ivec3(0, 0, 0), dimension, maxDepth, 0);
    }

    template<typename T>
    void SVO<T>::setVoxel(const glm::ivec3& pos, T value)
    {
        assert(pos[0] >= 0 && pos[1] >= 0 && pos[2] >= 0);
        assert(pos[0] < static_cast<int>(dimension) && pos[1] < static_cast<int>(dimension) && pos[2] < static_cast<int>(dimension));
        setVoxel(pos, value, 0, glm::ivec3(0, 0, 0), dimension);
    }

    template<typename T>
    T SVO<T>::getVoxel(const glm::ivec3& target, uint32_t currentNode, const glm::ivec3& currentOffset, uint32_t currentDimension, int maxDepth, int depth) const
    {
        auto& current = nodes[currentNode];
        if (current.childPointer == 0xFFFFFFFF || ((maxDepth != -1) && (maxDepth <= depth)))
        {
            return current.value;
        }
        auto childIndex    = pickChild(target, currentOffset, currentDimension);
        auto offset        = nextOffset(childIndex, currentOffset, currentDimension);
        auto nextDimension = currentDimension / 2;
        auto nextNodeIndex = current.childPointer + childIndex;
        return getVoxel(target, nextNodeIndex, offset, nextDimension, maxDepth, depth + 1);
    }

    template<typename T>
    T SVO<T>::average(const std::array<T, 8>& inputs)
    {
        assert(mergeStrategy != nullptr);
        return mergeStrategy->merge(inputs);
    }

    template<typename T>
    bool SVO<T>::buildFromGrid(const std::vector<T>& grid, uint32_t currentNode, const glm::ivec3& currentOffset, uint32_t currentDimension)
    {
        if (currentDimension == 2)
        {
            size_t x0        = static_cast<size_t>(currentOffset.x);
            size_t y0        = static_cast<size_t>(currentOffset.y);
            size_t z0        = static_cast<size_t>(currentOffset.z);
            size_t dim       = static_cast<size_t>(dimension);
            size_t sliceSize = dim * dim;

            T    val0    = grid[x0 + y0 * dim + z0 * sliceSize];
            bool allSame = (val0 == grid[(x0 + 1) + y0 * dim + z0 * sliceSize]) && (val0 == grid[x0 + (y0 + 1) * dim + z0 * sliceSize]) && (val0 == grid[(x0 + 1) + (y0 + 1) * dim + z0 * sliceSize]) &&
                           (val0 == grid[x0 + y0 * dim + (z0 + 1) * sliceSize]) && (val0 == grid[(x0 + 1) + y0 * dim + (z0 + 1) * sliceSize]) &&
                           (val0 == grid[x0 + (y0 + 1) * dim + (z0 + 1) * sliceSize]) && (val0 == grid[(x0 + 1) + (y0 + 1) * dim + (z0 + 1) * sliceSize]);

            if (allSame)
            {
                nodes[currentNode].value        = val0;
                nodes[currentNode].childPointer = 0xFFFFFFFF;
                return true;
            }

            uint32_t childBlock             = addNodeBlock(currentNode);
            nodes[currentNode].childPointer = childBlock;

            for (short i = 0; i < 8; i++)
            {
                glm::ivec3 childOffset             = nextOffset(i, currentOffset, 2);
                size_t     idx                     = static_cast<size_t>(childOffset.x) + static_cast<size_t>(childOffset.y) * dim + static_cast<size_t>(childOffset.z) * sliceSize;
                nodes[childBlock + i].value        = grid[idx];
                nodes[childBlock + i].childPointer = 0xFFFFFFFF;
            }

            std::array<T, 8> toAverage;
            for (size_t av = 0; av < 8; av++)
            {
                toAverage[av] = nodes[childBlock + av].value;
            }
            nodes[currentNode].value = average(toAverage);
            return false;
        }

        uint32_t childBlock             = addNodeBlock(currentNode);
        nodes[currentNode].childPointer = childBlock;
        uint32_t half                   = currentDimension / 2;

        bool allChildrenAreLeaves = true;
        for (short i = 0; i < 8; i++)
        {
            glm::ivec3 childOffset    = nextOffset(i, currentOffset, currentDimension);
            uint32_t   childNodeIndex = childBlock + i;
            bool       childIsLeaf    = buildFromGrid(grid, childNodeIndex, childOffset, half);
            if (!childIsLeaf)
            {
                allChildrenAreLeaves = false;
            }
        }

        if (allChildrenAreLeaves)
        {
            T    firstVal = nodes[childBlock].value;
            bool allSame  = true;
            for (size_t i = 1; i < 8; i++)
            {
                if (nodes[childBlock + i].value != firstVal)
                {
                    allSame = false;
                    break;
                }
            }

            if (allSame)
            {
                nodes[currentNode].value = firstVal;
                removeNodeBlock(currentNode);
                return true;
            }
        }

        std::array<T, 8> toAverage;
        for (size_t av = 0; av < 8; av++)
        {
            toAverage[av] = nodes[childBlock + av].value;
        }
        nodes[currentNode].value = average(toAverage);
        return false;
    }

    template<typename T>
    bool SVO<T>::isHomogenous(const glm::ivec3& start, const glm::ivec3& size) const
    {
        if (size.x <= 0 || size.y <= 0 || size.z <= 0)
        {
            return true;
        }

        auto intersects = [](const glm::ivec3& b1_start, const glm::ivec3& b1_size, const glm::ivec3& b2_start, const glm::ivec3& b2_size)
        {
            return (b1_start.x < b2_start.x + b2_size.x && b1_start.x + b1_size.x > b2_start.x) && (b1_start.y < b2_start.y + b2_size.y && b1_start.y + b1_size.y > b2_start.y) &&
                   (b1_start.z < b2_start.z + b2_size.z && b1_start.z + b1_size.z > b2_start.z);
        };

        std::optional<T> firstValue;

        auto checkNode = [&](auto& self, uint32_t nodeIdx, glm::ivec3 nodeOffset, uint32_t nodeDim) -> bool
        {
            if (!intersects(nodeOffset, glm::ivec3(nodeDim), start, size))
            {
                return true;
            }

            const auto& node = nodes[nodeIdx];

            if (node.childPointer == 0xFFFFFFFF)
            {
                if (!firstValue.has_value())
                {
                    firstValue = node.value;
                    return true;
                }
                return node.value == firstValue.value();
            }

            uint32_t half = nodeDim / 2;
            for (short i = 0; i < 8; ++i)
            {
                glm::ivec3 childOff = nextOffset(i, nodeOffset, nodeDim);
                if (!self(self, node.childPointer + i, childOff, half))
                {
                    return false;
                }
            }

            return true;
        };

        return checkNode(checkNode, 0, glm::ivec3(0), dimension);
    }

    template<typename T>
    void SVO<T>::get3DArray(std::vector<T>& outBuffer, const glm::ivec3& start, const glm::ivec3& size, int targetDepth) const
    {
        int      maxDepth = 0;
        uint32_t tempDim  = dimension;
        while (tempDim >>= 1)
        {
            ++maxDepth;
        }
        if (targetDepth == -1)
        {
            targetDepth = maxDepth;
        }
        int scale = 1 << (maxDepth - targetDepth);

        glm::ivec3 outRes = size / scale;
        outRes            = glm::max(outRes, glm::ivec3(0));

        size_t totalCells = static_cast<size_t>(outRes.x) * outRes.y * outRes.z;

        if (outBuffer.size() != totalCells)
        {
            outBuffer.assign(totalCells, T{});
        }
        else
        {
            std::fill(outBuffer.begin(), outBuffer.end(), T{});
        }

        if (totalCells == 0)
        {
            return;
        }

        auto fillBufferArea = [&](const T& value, const glm::ivec3& iStart, const glm::ivec3& iEnd)
        {
            int bStartX = (iStart.x - start.x) / scale;
            int bStartY = (iStart.y - start.y) / scale;
            int bStartZ = (iStart.z - start.z) / scale;

            int bEndX = (iEnd.x - start.x + scale - 1) / scale;
            int bEndY = (iEnd.y - start.y + scale - 1) / scale;
            int bEndZ = (iEnd.z - start.z + scale - 1) / scale;

            bEndX = std::min(bEndX, outRes.x);
            bEndY = std::min(bEndY, outRes.y);
            bEndZ = std::min(bEndZ, outRes.z);

            for (int z = bStartZ; z < bEndZ; ++z)
            {
                int offZ = z * outRes.x * outRes.y;
                for (int y = bStartY; y < bEndY; ++y)
                {
                    int offY = y * outRes.x + offZ;
                    for (int x = bStartX; x < bEndX; ++x)
                    {
                        outBuffer[x + offY] = value;
                    }
                }
            }
        };

        auto traverseWalk = [&](auto& self, uint32_t nodeIdx, glm::ivec3 nodeOff, uint32_t nodeDim, int currentDepth) -> void
        {
            glm::ivec3 nodeEnd = nodeOff + glm::ivec3(nodeDim);
            glm::ivec3 iStart  = glm::max(start, nodeOff);
            glm::ivec3 iEnd    = glm::min(start + size, nodeEnd);

            if (iStart.x >= iEnd.x || iStart.y >= iEnd.y || iStart.z >= iEnd.z)
            {
                return;
            }

            const auto& node = nodes[nodeIdx];

            if (currentDepth >= targetDepth || node.childPointer == 0xFFFFFFFF)
            {
                fillBufferArea(node.value, iStart, iEnd);
                return;
            }

            uint32_t half = nodeDim >> 1;
            for (uint32_t i = 0; i < 8; ++i)
            {
                glm::ivec3 cOff = nodeOff;
                if (i & 1)
                {
                    cOff.x += half;
                }
                if (i & 2)
                {
                    cOff.y += half;
                }
                if (i & 4)
                {
                    cOff.z += half;
                }

                self(self, node.childPointer + i, cOff, half, currentDepth + 1);
            }
        };

        traverseWalk(traverseWalk, 0, glm::ivec3(0), dimension, 0);
    }

    template<typename T>
    template<typename Visitor>
    void SVO<T>::traverse(Visitor&& visitor) const
    {
        auto walk = [&](auto& self, uint32_t nodeIdx, glm::ivec3 nodeOffset, uint32_t nodeDim, int depth) -> void
        {
            const auto& node   = nodes[nodeIdx];
            bool        isLeaf = (node.childPointer == 0xFFFFFFFF);

            if constexpr (std::is_invocable_r_v<bool, Visitor, const SVONode<T>&, glm::ivec3, uint32_t, int, bool>)
            {
                bool descend = visitor(node, nodeOffset, nodeDim, depth, isLeaf);
                if (!descend || isLeaf)
                {
                    return;
                }
            }
            else
            {
                visitor(node, nodeOffset, nodeDim, depth, isLeaf);
                if (isLeaf)
                {
                    return;
                }
            }

            uint32_t half = nodeDim >> 1;
            for (short i = 0; i < 8; ++i)
            {
                glm::ivec3 childOff = nextOffset(i, nodeOffset, nodeDim);
                self(self, node.childPointer + i, childOff, half, depth + 1);
            }
        };

        walk(walk, 0, glm::ivec3(0), dimension, 0);
    }

    template<typename T>
    template<typename LeafVisitor>
    void SVO<T>::traverseLeaves(LeafVisitor&& leafVisitor) const
    {
        traverse(
          [&](const SVONode<T>& node, const glm::ivec3& offset, uint32_t nodeDim, int depth, bool isLeaf)
          {
              if (isLeaf)
              {
                  leafVisitor(node, offset, nodeDim, depth);
                  return false;
              }
              return true;
          });
    }

    template<typename T>
    void SVO<T>::deleteSubtree(uint32_t nodeIndex)
    {
        uint32_t childPtr = nodes[nodeIndex].childPointer;
        if (childPtr == 0xFFFFFFFF)
        {
            return;
        }

        for (uint32_t i = 0; i < 8; ++i)
        {
            deleteSubtree(childPtr + i);
        }
        removeNodeBlock(nodeIndex);
    }

    template<typename T>
    std::unique_ptr<SVO<T>> SVO<T>::getSubSVO(const glm::ivec3& offset, uint32_t subDimension) const
    {
        assert(subDimension > 0 && (subDimension & (subDimension - 1)) == 0);
        assert(subDimension <= dimension);
        assert(offset.x >= 0 && offset.y >= 0 && offset.z >= 0);
        assert(offset.x + static_cast<int>(subDimension) <= static_cast<int>(dimension) && offset.y + static_cast<int>(subDimension) <= static_cast<int>(dimension) &&
               offset.z + static_cast<int>(subDimension) <= static_cast<int>(dimension));
        assert(offset.x % subDimension == 0 && offset.y % subDimension == 0 && offset.z % subDimension == 0);

        if (subDimension == 0 || (subDimension & (subDimension - 1)) != 0 || subDimension > dimension)
        {
            return nullptr;
        }
        if (offset.x < 0 || offset.y < 0 || offset.z < 0 || offset.x + static_cast<int>(subDimension) > static_cast<int>(dimension) ||
            offset.y + static_cast<int>(subDimension) > static_cast<int>(dimension) || offset.z + static_cast<int>(subDimension) > static_cast<int>(dimension) || offset.x % subDimension != 0 ||
            offset.y % subDimension != 0 || offset.z % subDimension != 0)
        {
            return nullptr;
        }

        uint32_t   currentNode = 0;
        glm::ivec3 currentOffset(0);
        uint32_t   currentDimension = dimension;

        while (currentDimension > subDimension)
        {
            if (nodes[currentNode].childPointer == 0xFFFFFFFF)
            {
                return std::make_unique<SVO<T>>(subDimension, nodes[currentNode].value, *this->mergeStrategy);
            }

            short childIndex = pickChild(offset, currentOffset, currentDimension);
            currentOffset    = nextOffset(childIndex, currentOffset, currentDimension);
            currentDimension /= 2;
            currentNode = nodes[currentNode].childPointer + childIndex;
        }

        if (nodes[currentNode].childPointer == 0xFFFFFFFF)
        {
            return std::make_unique<SVO<T>>(subDimension, nodes[currentNode].value, *this->mergeStrategy);
        }

        auto subTree = std::make_unique<SVO<T>>(subDimension, nodes[currentNode].value, *this->mergeStrategy);

        auto copySubtree = [&](auto& self, uint32_t srcNodeIdx, uint32_t dstNodeIdx) -> void
        {
            uint32_t srcChildBlock = nodes[srcNodeIdx].childPointer;
            if (srcChildBlock == 0xFFFFFFFF)
            {
                return;
            }

            uint32_t dstChildBlock                  = subTree->addNodeBlock(dstNodeIdx);
            subTree->nodes[dstNodeIdx].childPointer = dstChildBlock;

            for (short i = 0; i < 8; ++i)
            {
                subTree->nodes[dstChildBlock + i].value        = nodes[srcChildBlock + i].value;
                subTree->nodes[dstChildBlock + i].childPointer = 0xFFFFFFFF;
                self(self, srcChildBlock + i, dstChildBlock + i);
            }
        };

        copySubtree(copySubtree, currentNode, 0);

        return subTree;
    }

    template<typename T>
    void SVO<T>::setSubSVO(const glm::ivec3& offset, const std::unique_ptr<SVO<T>>& subSVO)
    {
        if (subSVO)
        {
            setSubSVO(offset, *subSVO);
        }
    }

    template<typename T>
    void SVO<T>::setSubSVO(const glm::ivec3& offset, const SVO<T>& subSVO)
    {
        assert(this->mergeStrategy == subSVO.mergeStrategy || typeid(*this->mergeStrategy) == typeid(*subSVO.mergeStrategy));

        uint32_t subDimension = subSVO.getDimension();
        assert(subDimension > 0 && (subDimension & (subDimension - 1)) == 0);
        assert(subDimension <= dimension);
        assert(offset.x >= 0 && offset.y >= 0 && offset.z >= 0);
        assert(offset.x + static_cast<int>(subDimension) <= static_cast<int>(dimension) && offset.y + static_cast<int>(subDimension) <= static_cast<int>(dimension) &&
               offset.z + static_cast<int>(subDimension) <= static_cast<int>(dimension));
        assert(offset.x % subDimension == 0 && offset.y % subDimension == 0 && offset.z % subDimension == 0);

        if (subDimension == 0 || (subDimension & (subDimension - 1)) != 0 || subDimension > dimension)
        {
            return;
        }
        if (offset.x < 0 || offset.y < 0 || offset.z < 0 || offset.x + static_cast<int>(subDimension) > static_cast<int>(dimension) ||
            offset.y + static_cast<int>(subDimension) > static_cast<int>(dimension) || offset.z + static_cast<int>(subDimension) > static_cast<int>(dimension) || offset.x % subDimension != 0 ||
            offset.y % subDimension != 0 || offset.z % subDimension != 0)
        {
            return;
        }

        if (&subSVO == this && offset == glm::ivec3(0))
        {
            return;
        }

        uint32_t   currentNode = 0;
        glm::ivec3 currentOffset(0);
        uint32_t   currentDimension = dimension;

        if (nodes[0].childPointer == 0xFFFFFFFF && subSVO.nodes[0].childPointer == 0xFFFFFFFF && nodes[0].value == subSVO.nodes[0].value)
        {
            return;
        }

        std::vector<uint32_t> ancestorPath;
        ancestorPath.reserve(32);

        while (currentDimension > subDimension)
        {
            if (nodes[currentNode].childPointer == 0xFFFFFFFF && subSVO.nodes[0].childPointer == 0xFFFFFFFF && nodes[currentNode].value == subSVO.nodes[0].value)
            {
                return;
            }

            ancestorPath.push_back(currentNode);

            if (nodes[currentNode].childPointer == 0xFFFFFFFF)
            {
                uint32_t newBlock               = addNodeBlock(currentNode);
                nodes[currentNode].childPointer = newBlock;
            }

            short childIndex = pickChild(offset, currentOffset, currentDimension);
            currentOffset    = nextOffset(childIndex, currentOffset, currentDimension);
            currentDimension /= 2;
            currentNode = nodes[currentNode].childPointer + childIndex;
        }

        deleteSubtree(currentNode);

        nodes[currentNode].value        = subSVO.nodes[0].value;
        nodes[currentNode].childPointer = 0xFFFFFFFF;

        if (subSVO.nodes[0].childPointer != 0xFFFFFFFF)
        {
            auto cloneSubtree = [&](auto& self, uint32_t dstNodeIdx, uint32_t srcNodeIdx) -> void
            {
                uint32_t srcChildBlock = subSVO.nodes[srcNodeIdx].childPointer;
                if (srcChildBlock == 0xFFFFFFFF)
                {
                    return;
                }

                uint32_t dstChildBlock         = addNodeBlock(dstNodeIdx);
                nodes[dstNodeIdx].childPointer = dstChildBlock;

                for (short i = 0; i < 8; ++i)
                {
                    nodes[dstChildBlock + i].value        = subSVO.nodes[srcChildBlock + i].value;
                    nodes[dstChildBlock + i].childPointer = 0xFFFFFFFF;
                    self(self, dstChildBlock + i, srcChildBlock + i);
                }
            };

            cloneSubtree(cloneSubtree, currentNode, 0);
        }

        for (auto it = ancestorPath.rbegin(); it != ancestorPath.rend(); ++it)
        {
            uint32_t ancNode      = *it;
            uint32_t childPointer = nodes[ancNode].childPointer;
            assert(childPointer != 0xFFFFFFFF);

            T    val         = nodes[childPointer].value;
            bool canCollapse = true;
            for (size_t i = 0; i < 8; i++)
            {
                if (val != nodes[childPointer + i].value || nodes[childPointer + i].childPointer != 0xFFFFFFFF)
                {
                    canCollapse = false;
                    break;
                }
            }

            if (canCollapse)
            {
                nodes[ancNode].value = val;
                removeNodeBlock(ancNode);
            }
            else
            {
                std::array<T, 8> toAverage;
                for (size_t av = 0; av < 8; av++)
                {
                    toAverage[av] = nodes[childPointer + av].value;
                }
                nodes[ancNode].value = average(toAverage);
            }
        }
    }
}

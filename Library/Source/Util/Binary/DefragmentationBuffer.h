#pragma once

#include "BufferRange.h"
#include "BufferSpan.h"
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <span>
#include <unordered_map>
#include <vector>

namespace Library::Binary
{
    template<typename T>
    class DefragmentationBuffer
    {
      public:
        DefragmentationBuffer()  = default;
        ~DefragmentationBuffer() = default;

        size_t set(std::span<const T> package);
        size_t set(const std::vector<T>& package);
        size_t set(std::vector<T>&& package);
        size_t set(std::initializer_list<T> package);

        bool   remove(size_t id);
        void   clear();
        void   reserve(size_t dataCapacity, size_t spanCapacity = 0);

        const std::vector<T>&    getData() const;
        std::vector<BufferRange> popDirty();

        size_t size() const;

      private:
        static constexpr uint32_t NONE = UINT32_MAX;

        struct Span
        {
            size_t   offset         = 0;
            size_t   count          = 0;
            size_t   packageId      = 0;
            uint32_t prevInPackage  = NONE;
            uint32_t nextInPackage  = NONE;
        };

        uint32_t allocSpan(size_t offset, size_t count, size_t id);
        void     unlinkAndFree(uint32_t s);
        void     insertPiece(size_t offset, size_t count, size_t id);
        void     fillHole(size_t holeOffset, size_t holeCount);
        void     markDirty(size_t offset, size_t count);
        void     coalesceDirtyRanges();

        std::vector<T>                       data;
        std::vector<Span>                    spans;
        std::vector<uint32_t>                freeSlots;
        std::unordered_map<size_t, uint32_t> byStart;
        std::unordered_map<size_t, uint32_t> byEnd;
        std::unordered_map<size_t, uint32_t> packages;
        std::vector<BufferRange>             dirtyRanges;
        std::vector<BufferRange>             holeScratchBuffer;
        size_t                               nextPackageId = 1;
    };

    template<typename T>
    size_t DefragmentationBuffer<T>::set(std::span<const T> package)
    {
        if (package.empty())
        {
            return 0;
        }

        size_t id           = nextPackageId++;
        size_t insertOffset = data.size();
        size_t count        = package.size();

        data.insert(data.end(), package.begin(), package.end());
        allocSpan(insertOffset, count, id);
        markDirty(insertOffset, count);

        return id;
    }

    template<typename T>
    size_t DefragmentationBuffer<T>::set(const std::vector<T>& package)
    {
        return set(std::span<const T>(package.data(), package.size()));
    }

    template<typename T>
    size_t DefragmentationBuffer<T>::set(std::vector<T>&& package)
    {
        if (package.empty())
        {
            return 0;
        }

        size_t id           = nextPackageId++;
        size_t insertOffset = data.size();
        size_t count        = package.size();

        if (data.empty() && data.capacity() == 0)
        {
            data = std::move(package);
        }
        else
        {
            data.insert(data.end(), std::make_move_iterator(package.begin()), std::make_move_iterator(package.end()));
        }

        allocSpan(insertOffset, count, id);
        markDirty(insertOffset, count);

        return id;
    }

    template<typename T>
    size_t DefragmentationBuffer<T>::set(std::initializer_list<T> package)
    {
        return set(std::span<const T>(package.begin(), package.size()));
    }

    template<typename T>
    bool DefragmentationBuffer<T>::remove(size_t id)
    {
        if (id == 0)
        {
            return false;
        }

        auto pkgIt = packages.find(id);
        if (pkgIt == packages.end())
        {
            return false;
        }

        holeScratchBuffer.clear();
        uint32_t s = pkgIt->second;
        while (s != NONE)
        {
            const Span& sp = spans[s];
            holeScratchBuffer.emplace_back(sp.offset, sp.count);
            byStart.erase(sp.offset);
            byEnd.erase(sp.offset + sp.count);
            uint32_t next = sp.nextInPackage;
            freeSlots.push_back(s);
            s = next;
        }
        packages.erase(pkgIt);

        if (holeScratchBuffer.empty())
        {
            return false;
        }

        std::sort(holeScratchBuffer.begin(), holeScratchBuffer.end(), [](const BufferRange& a, const BufferRange& b) {
            return a.offset > b.offset;
        });

        for (const auto& hole : holeScratchBuffer)
        {
            fillHole(hole.offset, hole.count);
        }

        return true;
    }

    template<typename T>
    void DefragmentationBuffer<T>::clear()
    {
        data.clear();
        spans.clear();
        freeSlots.clear();
        byStart.clear();
        byEnd.clear();
        packages.clear();
        dirtyRanges.clear();
        holeScratchBuffer.clear();
        nextPackageId = 1;
    }

    template<typename T>
    void DefragmentationBuffer<T>::reserve(size_t dataCapacity, size_t spanCapacity)
    {
        data.reserve(dataCapacity);
        if (spanCapacity > 0)
        {
            spans.reserve(spanCapacity);
            freeSlots.reserve(spanCapacity);
            byStart.reserve(spanCapacity);
            byEnd.reserve(spanCapacity);
            packages.reserve(spanCapacity);
        }
    }

    template<typename T>
    const std::vector<T>& DefragmentationBuffer<T>::getData() const
    {
        return data;
    }

    template<typename T>
    std::vector<BufferRange> DefragmentationBuffer<T>::popDirty()
    {
        coalesceDirtyRanges();
        std::vector<BufferRange> result = std::move(dirtyRanges);
        dirtyRanges.clear();
        return result;
    }

    template<typename T>
    size_t DefragmentationBuffer<T>::size() const
    {
        return data.size();
    }

    template<typename T>
    uint32_t DefragmentationBuffer<T>::allocSpan(size_t offset, size_t count, size_t id)
    {
        uint32_t s;
        if (!freeSlots.empty())
        {
            s = freeSlots.back();
            freeSlots.pop_back();
        }
        else
        {
            s = static_cast<uint32_t>(spans.size());
            spans.emplace_back();
        }

        auto [pkgIt, inserted] = packages.try_emplace(id, NONE);
        spans[s]               = Span{ offset, count, id, NONE, pkgIt->second };
        if (pkgIt->second != NONE)
        {
            spans[pkgIt->second].prevInPackage = s;
        }
        pkgIt->second         = s;
        byStart[offset]       = s;
        byEnd[offset + count] = s;
        return s;
    }

    template<typename T>
    void DefragmentationBuffer<T>::unlinkAndFree(uint32_t s)
    {
        Span& sp = spans[s];
        if (sp.prevInPackage != NONE)
        {
            spans[sp.prevInPackage].nextInPackage = sp.nextInPackage;
        }
        else if (sp.nextInPackage != NONE)
        {
            packages[sp.packageId] = sp.nextInPackage;
        }
        else
        {
            packages.erase(sp.packageId);
        }

        if (sp.nextInPackage != NONE)
        {
            spans[sp.nextInPackage].prevInPackage = sp.prevInPackage;
        }

        freeSlots.push_back(s);
    }

    template<typename T>
    void DefragmentationBuffer<T>::insertPiece(size_t offset, size_t count, size_t id)
    {
        uint32_t left  = NONE;
        uint32_t right = NONE;

        if (auto it = byEnd.find(offset); it != byEnd.end() && spans[it->second].packageId == id)
        {
            left = it->second;
        }
        if (auto it = byStart.find(offset + count); it != byStart.end() && spans[it->second].packageId == id)
        {
            right = it->second;
        }

        if (left != NONE && right != NONE)
        {
            Span& L = spans[left];
            Span& R = spans[right];
            byEnd.erase(offset);
            byStart.erase(offset + count);
            byEnd[R.offset + R.count] = left;
            L.count += count + R.count;
            unlinkAndFree(right);
        }
        else if (left != NONE)
        {
            Span& L = spans[left];
            byEnd.erase(offset);
            L.count += count;
            byEnd[offset + count] = left;
        }
        else if (right != NONE)
        {
            Span& R = spans[right];
            byStart.erase(offset + count);
            R.offset = offset;
            R.count += count;
            byStart[offset] = right;
        }
        else
        {
            allocSpan(offset, count, id);
        }
    }

    template<typename T>
    void DefragmentationBuffer<T>::fillHole(size_t holeOffset, size_t holeCount)
    {
        while (holeCount > 0)
        {
            if (data.size() <= holeOffset)
            {
                break;
            }

            auto tailIt = byEnd.find(data.size());
            if (tailIt == byEnd.end())
            {
                // Hole sits at the end of the buffer: just shrink.
                size_t shrink = std::min(holeCount, data.size() - holeOffset);
                data.erase(data.end() - shrink, data.end());
                holeCount -= shrink;
                if (shrink == 0)
                {
                    break;
                }
                continue;
            }

            uint32_t ts           = tailIt->second;
            size_t   moveCount    = std::min(holeCount, spans[ts].count);
            size_t   sourceOffset = data.size() - moveCount;

            if (sourceOffset != holeOffset)
            {
                std::move(data.begin() + sourceOffset, data.begin() + sourceOffset + moveCount, data.begin() + holeOffset);
                markDirty(holeOffset, moveCount);
            }
            data.erase(data.begin() + sourceOffset, data.end());

            byEnd.erase(tailIt);
            size_t tailId     = spans[ts].packageId;
            size_t tailOffset = spans[ts].offset;
            spans[ts].count -= moveCount;

            if (spans[ts].count == 0)
            {
                byStart.erase(tailOffset);
                unlinkAndFree(ts);
            }
            else
            {
                byEnd[tailOffset + spans[ts].count] = ts;
            }

            insertPiece(holeOffset, moveCount, tailId);
            holeOffset += moveCount;
            holeCount -= moveCount;
        }
    }

    template<typename T>
    void DefragmentationBuffer<T>::markDirty(size_t offset, size_t count)
    {
        if (count == 0)
        {
            return;
        }

        if (!dirtyRanges.empty())
        {
            BufferRange& last = dirtyRanges.back();
            if (last.offset + last.count == offset)
            {
                last.count += count;
                return;
            }
        }

        dirtyRanges.emplace_back(offset, count);
    }

    template<typename T>
    void DefragmentationBuffer<T>::coalesceDirtyRanges()
    {
        if (dirtyRanges.empty())
        {
            return;
        }

        if (data.empty())
        {
            dirtyRanges.clear();
            return;
        }

        const size_t maxSize  = data.size();
        size_t       writeIdx = 0;
        for (size_t readIdx = 0; readIdx < dirtyRanges.size(); ++readIdx)
        {
            if (dirtyRanges[readIdx].offset >= maxSize)
            {
                continue;
            }
            if (dirtyRanges[readIdx].offset + dirtyRanges[readIdx].count > maxSize)
            {
                dirtyRanges[readIdx].count = maxSize - dirtyRanges[readIdx].offset;
            }
            if (dirtyRanges[readIdx].count == 0)
            {
                continue;
            }
            if (writeIdx != readIdx)
            {
                dirtyRanges[writeIdx] = dirtyRanges[readIdx];
            }
            ++writeIdx;
        }
        dirtyRanges.resize(writeIdx);

        if (dirtyRanges.size() < 2)
        {
            return;
        }

        std::sort(dirtyRanges.begin(), dirtyRanges.end(), [](const BufferRange& a, const BufferRange& b) {
            if (a.offset != b.offset)
            {
                return a.offset < b.offset;
            }
            return a.count < b.count;
        });

        writeIdx = 0;
        for (size_t readIdx = 1; readIdx < dirtyRanges.size(); ++readIdx)
        {
            BufferRange& last    = dirtyRanges[writeIdx];
            size_t       lastEnd = last.offset + last.count;

            if (dirtyRanges[readIdx].offset <= lastEnd)
            {
                size_t currentEnd = dirtyRanges[readIdx].offset + dirtyRanges[readIdx].count;
                if (currentEnd > lastEnd)
                {
                    last.count = currentEnd - last.offset;
                }
            }
            else
            {
                ++writeIdx;
                if (writeIdx != readIdx)
                {
                    dirtyRanges[writeIdx] = dirtyRanges[readIdx];
                }
            }
        }
        dirtyRanges.resize(writeIdx + 1);
    }
}

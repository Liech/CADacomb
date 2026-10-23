#include "DefragmentationBuffer.h"

#ifdef ISTESTPROJECT
#include <catch2/catch_test_macros.hpp>
#include <random>

namespace Library::Binary
{
    TEST_CASE("DefragmentationBuffer/AddSinglePackage", "[DefragmentationBuffer]")
    {
        DefragmentationBuffer<int> buffer;
        CHECK(buffer.size() == 0);

        std::vector<int> pkg1 = { 10, 20, 30, 40, 50 };
        size_t           id1  = buffer.set(pkg1);

        CHECK(id1 == 1);
        CHECK(buffer.size() == 5);
        CHECK(buffer.getData() == pkg1);

        auto dirty = buffer.popDirty();
        REQUIRE(dirty.size() == 1);
        CHECK(dirty[0].offset == 0);
        CHECK(dirty[0].count == 5);

        CHECK(buffer.popDirty().empty());
    }

    TEST_CASE("DefragmentationBuffer/AddMultiplePackagesDense", "[DefragmentationBuffer]")
    {
        DefragmentationBuffer<int> buffer;

        size_t id1 = buffer.set({ 1, 2, 3 });
        size_t id2 = buffer.set({ 4, 5, 6, 7 });
        size_t id3 = buffer.set({ 8, 9 });

        CHECK(id1 == 1);
        CHECK(id2 == 2);
        CHECK(id3 == 3);
        CHECK(buffer.size() == 9);

        std::vector<int> expected = { 1, 2, 3, 4, 5, 6, 7, 8, 9 };
        CHECK(buffer.getData() == expected);
    }

    TEST_CASE("DefragmentationBuffer/UpdateViaRemoveAndSet", "[DefragmentationBuffer]")
    {
        DefragmentationBuffer<int> buffer;

        size_t id1 = buffer.set({ 1, 2 });
        size_t id2 = buffer.set({ 10, 20, 30 });
        size_t id3 = buffer.set({ 100, 200 });

        buffer.popDirty();

        buffer.remove(id2);
        size_t newId2 = buffer.set({ 77, 88, 99 });
        CHECK(newId2 == 4);

        CHECK(buffer.size() == 7);
        std::vector<int> expected = { 1, 2, 100, 200, 77, 88, 99 };
        CHECK(buffer.getData() == expected);
    }

    TEST_CASE("DefragmentationBuffer/RemoveTailPackageZeroMoves", "[DefragmentationBuffer]")
    {
        DefragmentationBuffer<int> buffer;

        size_t id1 = buffer.set({ 1, 2, 3 });
        size_t id2 = buffer.set({ 4, 5 });
        buffer.popDirty();

        bool removed = buffer.remove(id2);
        CHECK(removed);
        CHECK(buffer.size() == 3);
        CHECK(buffer.getData() == std::vector<int>{ 1, 2, 3 });

        CHECK(buffer.popDirty().empty());
    }

    TEST_CASE("DefragmentationBuffer/RemoveMiddlePackageTailFill", "[DefragmentationBuffer]")
    {
        DefragmentationBuffer<int> buffer;

        size_t id1 = buffer.set({ 1, 2, 3 });
        size_t id2 = buffer.set({ 10, 20, 30, 40 });
        size_t id4 = buffer.set({ 50, 60 });
        size_t id3 = buffer.set({ 100, 200, 300, 400, 500 });
        CHECK(buffer.size() == 14);

        buffer.popDirty();

        bool removed = buffer.remove(id2);
        CHECK(removed);
        CHECK(buffer.size() == 10);

        auto dirty = buffer.popDirty();
        REQUIRE(dirty.size() == 1);
        CHECK(dirty[0].offset == 3);
        CHECK(dirty[0].count == 4);

        std::vector<int> expected = { 1, 2, 3, 200, 300, 400, 500, 50, 60, 100 };
        CHECK(buffer.getData() == expected);
    }

    TEST_CASE("DefragmentationBuffer/RemoveSegmentedPackage", "[DefragmentationBuffer]")
    {
        DefragmentationBuffer<int> buffer;

        size_t id1 = buffer.set({ 1, 2, 3 });
        size_t id2 = buffer.set({ 10, 20, 30, 40 });
        size_t id4 = buffer.set({ 50, 60 });
        size_t id3 = buffer.set({ 100, 200, 300, 400, 500 });

        buffer.remove(id2);
        CHECK(buffer.size() == 10);

        buffer.popDirty();

        bool removed = buffer.remove(id3);
        CHECK(removed);
        CHECK(buffer.size() == 5);

        std::vector<int> expected = { 1, 2, 3, 50, 60 };
        CHECK(buffer.getData() == expected);
    }

    TEST_CASE("DefragmentationBuffer/AdjacentSpansMergeAutomatically", "[DefragmentationBuffer]")
    {
        DefragmentationBuffer<int> buffer;

        size_t id1 = buffer.set({ 1, 2, 3 });
        size_t id2 = buffer.set({ 10, 20, 30, 40 });
        size_t id3 = buffer.set({ 100, 200, 300, 400, 500 });
        CHECK(buffer.size() == 12);

        buffer.remove(id2);
        CHECK(buffer.size() == 8);

        std::vector<int> expected = { 1, 2, 3, 200, 300, 400, 500, 100 };
        CHECK(buffer.getData() == expected);
    }

    TEST_CASE("DefragmentationBuffer/CoalesceAdjacentDirtyRanges", "[DefragmentationBuffer]")
    {
        DefragmentationBuffer<int> buffer;

        buffer.set({ 1, 2 });
        buffer.set({ 3, 4 });
        buffer.set({ 5, 6 });

        auto dirty = buffer.popDirty();
        REQUIRE(dirty.size() == 1);
        CHECK(dirty[0].offset == 0);
        CHECK(dirty[0].count == 6);

        CHECK(buffer.popDirty().empty());
    }

    TEST_CASE("DefragmentationBuffer/DirtyRangesDoNotExceedBufferSizeAfterRemove", "[DefragmentationBuffer]")
    {
        DefragmentationBuffer<int> buffer;

        size_t id1 = buffer.set({ 1, 2, 3 });
        size_t id2 = buffer.set({ 4, 5 });

        buffer.remove(id2);

        CHECK(buffer.size() == 3);

        auto dirty = buffer.popDirty();
        REQUIRE_FALSE(dirty.empty());
        for (const auto& range : dirty)
        {
            CHECK(range.offset + range.count <= buffer.size());
        }
        REQUIRE(dirty.size() == 1);
        CHECK(dirty[0].offset == 0);
        CHECK(dirty[0].count == 3);
    }

    TEST_CASE("DefragmentationBuffer/SequentialRemovalUntilEmpty", "[DefragmentationBuffer]")
    {
        DefragmentationBuffer<int> buffer;
        std::vector<size_t>        ids;

        for (int i = 1; i <= 10; i++)
        {
            std::vector<int> pkg(i, i * 10);
            ids.push_back(buffer.set(pkg));
        }

        CHECK(buffer.size() == 55);

        for (size_t i = 0; i < ids.size(); i++)
        {
            buffer.remove(ids[i]);

            size_t expectedRemaining = 0;
            for (size_t j = i + 1; j < ids.size(); j++)
            {
                expectedRemaining += (j + 1);
            }
            CHECK(buffer.size() == expectedRemaining);
        }

        CHECK(buffer.size() == 0);
    }

    struct CustomPayload
    {
        float x   = 0.0f;
        float y   = 0.0f;
        int   tag = 0;

        bool operator==(const CustomPayload& o) const
        {
            return x == o.x && y == o.y && tag == o.tag;
        }
    };

    TEST_CASE("DefragmentationBuffer/CustomStructPayload", "[DefragmentationBuffer]")
    {
        DefragmentationBuffer<CustomPayload> buffer;

        std::vector<CustomPayload> p1 = {
            { 1.0f, 2.0f, 10 },
            { 3.0f, 4.0f, 20 }
        };
        std::vector<CustomPayload> p2 = {
            { 5.0f, 6.0f, 30 }
        };

        size_t id1 = buffer.set(p1);
        size_t id2 = buffer.set(p2);

        CHECK(id1 == 1);
        CHECK(id2 == 2);
        CHECK(buffer.size() == 3);

        buffer.remove(id1);
        CHECK(buffer.size() == 1);
        CHECK(buffer.getData()[0] == p2[0]);
    }

    TEST_CASE("DefragmentationBuffer/ZeroFragmentationIntegrityStressTest", "[DefragmentationBuffer]")
    {
        DefragmentationBuffer<int>         buffer;
        std::mt19937                       rng(12345);
        std::uniform_int_distribution<int> sizeDist(1, 15);
        std::uniform_int_distribution<int> valDist(1, 1000);

        std::vector<std::pair<size_t, size_t>> activePackages;

        for (int i = 0; i < 40; i++)
        {
            int              sz = sizeDist(rng);
            std::vector<int> pkg(sz);
            for (int& v : pkg)
            {
                v = valDist(rng);
            }
            size_t id = buffer.set(pkg);
            activePackages.push_back({ id, static_cast<size_t>(sz) });
        }

        for (int step = 0; step < 200; step++)
        {
            if (!activePackages.empty() && (rng() % 3 == 0))
            {
                std::uniform_int_distribution<size_t> idxDist(0, activePackages.size() - 1);
                size_t                                removeIdx = idxDist(rng);
                size_t                                removeId  = activePackages[removeIdx].first;

                buffer.remove(removeId);
                activePackages.erase(activePackages.begin() + removeIdx);
            }
            else
            {
                int              sz = sizeDist(rng);
                std::vector<int> pkg(sz);
                for (int& v : pkg)
                {
                    v = valDist(rng);
                }
                size_t id = buffer.set(pkg);
                activePackages.push_back({ id, static_cast<size_t>(sz) });
            }

            size_t totalExpected = 0;
            for (const auto& [id, sz] : activePackages)
            {
                totalExpected += sz;
            }
            CHECK(buffer.size() == totalExpected);
        }

        for (const auto& [id, sz] : activePackages)
        {
            buffer.remove(id);
        }

        CHECK(buffer.size() == 0);
    }

    TEST_CASE("DefragmentationBuffer/TrivialLayoutAndPODProperties", "[DefragmentationBuffer]")
    {
        static_assert(std::is_trivially_copyable_v<BufferRange>);
        static_assert(std::is_trivially_destructible_v<BufferRange>);
        static_assert(sizeof(BufferRange) == 2 * sizeof(size_t));

        static_assert(std::is_trivially_copyable_v<BufferSpan>);
        static_assert(std::is_trivially_destructible_v<BufferSpan>);
        static_assert(sizeof(BufferSpan) == 3 * sizeof(size_t));

        CHECK(sizeof(BufferRange) == 16);
        CHECK(sizeof(BufferSpan) == 24);
    }

    TEST_CASE("DefragmentationBuffer/SpanAndMoveOverloads", "[DefragmentationBuffer]")
    {
        DefragmentationBuffer<int> buffer;

        int rawArray[] = { 10, 20, 30 };
        size_t id1 = buffer.set(std::span<const int>(rawArray, 3));
        CHECK(id1 == 1);
        CHECK(buffer.size() == 3);

        std::vector<int> vec = { 40, 50 };
        size_t id2 = buffer.set(std::move(vec));
        CHECK(id2 == 2);
        CHECK(buffer.size() == 5);

        std::vector<int> expected = { 10, 20, 30, 40, 50 };
        CHECK(buffer.getData() == expected);
    }

    TEST_CASE("DefragmentationBuffer/ReserveCapacity", "[DefragmentationBuffer]")
    {
        DefragmentationBuffer<int> buffer;
        buffer.reserve(100, 10);

        CHECK(buffer.getData().capacity() >= 100);

        size_t id1 = buffer.set({ 1, 2, 3 });
        size_t id2 = buffer.set({ 4, 5, 6 });
        CHECK(buffer.size() == 6);
        CHECK(buffer.getData().capacity() >= 100);

        buffer.remove(id1);
        CHECK(buffer.size() == 3);
        CHECK(buffer.getData() == std::vector<int>{ 4, 5, 6 });
    }

    struct MoveTracker
    {
        int value = 0;
        static inline int copyAssignments = 0;
        static inline int moveAssignments = 0;

        MoveTracker() = default;
        MoveTracker(int v) : value(v) {}
        MoveTracker(const MoveTracker& o) : value(o.value) {}
        MoveTracker& operator=(const MoveTracker& o)
        {
            value = o.value;
            ++copyAssignments;
            return *this;
        }
        MoveTracker(MoveTracker&& o) noexcept : value(o.value) {}
        MoveTracker& operator=(MoveTracker&& o) noexcept
        {
            value = o.value;
            ++moveAssignments;
            return *this;
        }
        bool operator==(const MoveTracker& o) const { return value == o.value; }
    };

    TEST_CASE("DefragmentationBuffer/MoveSemanticsDuringFillHole", "[DefragmentationBuffer]")
    {
        DefragmentationBuffer<MoveTracker> buffer;
        MoveTracker::copyAssignments = 0;
        MoveTracker::moveAssignments = 0;

        buffer.set({ MoveTracker(1), MoveTracker(2) });
        size_t id2 = buffer.set({ MoveTracker(3), MoveTracker(4) });
        buffer.set({ MoveTracker(5), MoveTracker(6) });

        int copiesBefore = MoveTracker::copyAssignments;
        int movesBefore  = MoveTracker::moveAssignments;

        buffer.remove(id2);

        CHECK(MoveTracker::copyAssignments == copiesBefore);
        CHECK(MoveTracker::moveAssignments > movesBefore);
        CHECK(buffer.size() == 4);
        CHECK(buffer.getData()[2].value == 5);
        CHECK(buffer.getData()[3].value == 6);
    }
}
#endif

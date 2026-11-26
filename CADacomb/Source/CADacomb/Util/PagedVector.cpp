#include "PagedVector.h"

#ifdef ISTESTPROJECT
#include <catch2/catch_test_macros.hpp>

namespace CADacomb
{
    TEST_CASE("PagedVector/PushBackAndIndexAcrossPages")
    {
        PagedVector<uint32_t, 4> vec;
        CHECK(vec.empty());
        CHECK(vec.size() == 0);
        CHECK(vec.capacity() == 0);

        for (uint32_t i = 0; i < 100; i++)
        {
            vec.push_back(i * 3);
        }

        REQUIRE(vec.size() == 100);
        CHECK(vec.pageCount() == 7);
        CHECK(vec.back() == 297);

        for (uint32_t i = 0; i < 100; i++)
        {
            CHECK(vec[i] == i * 3);
        }
    }

    TEST_CASE("PagedVector/PointerStabilityAcrossGrowth")
    {
        PagedVector<int, 4> vec;
        vec.push_back(42);
        const int* firstPtr = &vec[0];

        for (int i = 0; i < 500; i++)
        {
            vec.push_back(i);
        }

        CHECK(&vec[0] == firstPtr);
        CHECK(*firstPtr == 42);
    }

    TEST_CASE("PagedVector/CopyMoveAndShrink")
    {
        PagedVector<int, 4> vec;
        for (int i = 0; i < 64; i++)
        {
            vec.push_back(i);
        }

        PagedVector<int, 4> copy(vec);
        REQUIRE(copy.size() == 64);
        CHECK(copy[63] == 63);

        vec.resize(10);
        vec.shrinkToFit();
        CHECK(vec.size() == 10);
        CHECK(vec.pageCount() == 1);

        PagedVector<int, 4> moved(std::move(copy));
        CHECK(moved.size() == 64);
        CHECK(copy.empty());
    }
}
#endif

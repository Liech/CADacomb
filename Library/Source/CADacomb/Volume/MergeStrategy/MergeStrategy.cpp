#include "MergeStrategy.h"
#include "MeanMergeStrategy.h"
#include "MostPrevalentExceptMergeStrategy.h"
#include "MostPrevalentMergeStrategy.h"

#ifdef ISTESTPROJECT
#include <array>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <glm/glm.hpp>

namespace Library::Volume
{
    TEST_CASE("MeanMergeStrategy: scalar types", "[MergeStrategy]")
    {
        const auto& strategy = MeanMergeStrategy<float>::instance();

        std::array<float, 8> arr8 = { 1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f };
        CHECK(strategy.merge(arr8) == Catch::Approx(4.5f));

        const auto&        intStrategy = MeanMergeStrategy<int>::instance();
        std::array<int, 8> intArr      = { 1, 2, 3, 4, 1, 2, 3, 4 };
        CHECK(intStrategy.merge(intArr) == 2);
    }

    TEST_CASE("MeanMergeStrategy: vector types", "[MergeStrategy]")
    {
        const auto& strategy = MeanMergeStrategy<glm::vec3>::instance();

        std::array<glm::vec3, 8> arr    = { glm::vec3(1.0f, 2.0f, 3.0f), glm::vec3(3.0f, 4.0f, 5.0f), glm::vec3(1.0f, 2.0f, 3.0f), glm::vec3(3.0f, 4.0f, 5.0f),
                                            glm::vec3(1.0f, 2.0f, 3.0f), glm::vec3(3.0f, 4.0f, 5.0f), glm::vec3(1.0f, 2.0f, 3.0f), glm::vec3(3.0f, 4.0f, 5.0f) };
        glm::vec3                result = strategy.merge(arr);
        CHECK(result.x == Catch::Approx(2.0f));
        CHECK(result.y == Catch::Approx(3.0f));
        CHECK(result.z == Catch::Approx(4.0f));
    }

    TEST_CASE("MostPrevalentMergeStrategy: basic and tie-breaking", "[MergeStrategy]")
    {
        const auto& strategy = MostPrevalentMergeStrategy<int>::instance();

        std::array<int, 8> inputs = { 1, 2, 2, 3, 1, 4, 5, 6 };
        CHECK(strategy.merge(inputs) == 1);

        std::array<int, 8> inputs2 = { 7, 7, 7, 4, 1, 2, 3, 5 };
        CHECK(strategy.merge(inputs2) == 7);
    }

    TEST_CASE("MostPrevalentExceptMergeStrategy: excludes specified value", "[MergeStrategy]")
    {
        const auto& strategy = MostPrevalentExceptMergeStrategy<int, 0>::instance();

        std::array<int, 8> inputs = { 0, 0, 0, 0, 0, 0, 2, 2 };
        CHECK(strategy.merge(inputs) == 2);

        std::array<int, 8> allZero = { 0, 0, 0, 0, 0, 0, 0, 0 };
        CHECK(strategy.merge(allZero) == 0);
    }

    TEST_CASE("MergeStrategy: polymorphic interface", "[MergeStrategy]")
    {
        const MergeStrategy<float>& baseRef = MeanMergeStrategy<float>::instance();
        std::array<float, 8>        vals    = { 10.0f, 20.0f, 30.0f, 40.0f, 10.0f, 20.0f, 30.0f, 40.0f };
        CHECK(baseRef.merge(vals) == Catch::Approx(25.0f));
    }
}
#endif

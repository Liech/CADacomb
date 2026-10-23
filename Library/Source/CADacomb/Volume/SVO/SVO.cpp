#include "SVO.h"

namespace Library::Volume
{
}

#ifdef ISTESTPROJECT
#include "CADacomb/Volume/MergeStrategy/MeanMergeStrategy.h"
#include "CADacomb/Volume/MergeStrategy/MostPrevalentMergeStrategy.h"
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

using namespace Library::Volume;

TEST_CASE("SVO Initial State", "[SVO]")
{
    SVO<int> svo(8, 0);

    REQUIRE(svo.getVoxel({ 0, 0, 0 }) == 0);
    REQUIRE(svo.getVoxel({ 7, 7, 7 }) == 0);
}

TEST_CASE("SVO Single Voxel Storage", "[SVO]")
{
    SVO<int> svo(128, 0);

    svo.setVoxel({ 10, 20, 30 }, 99);

    REQUIRE(svo.getVoxel({ 10, 20, 30 }) == 99);
    REQUIRE(svo.getVoxel({ 10, 20, 31 }) == 0);
}

TEST_CASE("SVO Overwrite Logic", "[SVO]")
{
    SVO<int> svo(16, 5);

    svo.setVoxel({ 1, 1, 1 }, 10);
    REQUIRE(svo.getVoxel({ 1, 1, 1 }) == 10);

    svo.setVoxel({ 1, 1, 1 }, 20);
    REQUIRE(svo.getVoxel({ 1, 1, 1 }) == 20);
}

TEST_CASE("SVO Sparse Memory Usage", "[SVO]")
{
    SVO<uint8_t> svo(1024, 0);

    svo.setVoxel({ 0, 0, 0 }, 255);
    svo.setVoxel({ 1023, 1023, 1023 }, 255);

    REQUIRE(svo.getVoxel({ 0, 0, 0 }) == 255);
    REQUIRE(svo.getVoxel({ 1023, 1023, 1023 }) == 255);
}

TEST_CASE("SVO Mean Strategy Logic", "[SVO][Averaging]")
{
    SVO<float> svo(2, 0.0f, MeanMergeStrategy<float>::instance());

    svo.setVoxel({ 0, 0, 0 }, 10.0f);
    svo.setVoxel({ 1, 0, 0 }, 10.0f);
    svo.setVoxel({ 0, 0, 1 }, 10.0f);
    svo.setVoxel({ 1, 0, 1 }, 10.0f);

    REQUIRE(svo.getVoxel({ 0, 0, 0 }) == 10.0f);
}

TEST_CASE("SVO Most Prevalent Strategy Logic", "[SVO][Averaging]")
{
    SVO<int> svo(2, 0, MostPrevalentMergeStrategy<int>::instance());

    svo.setVoxel({ 0, 0, 0 }, 1);
    svo.setVoxel({ 1, 0, 0 }, 1);
    svo.setVoxel({ 0, 1, 0 }, 1);
    svo.setVoxel({ 1, 1, 0 }, 1);
    svo.setVoxel({ 0, 0, 1 }, 1);

    svo.setVoxel({ 1, 1, 1 }, 2);
    svo.setVoxel({ 0, 1, 1 }, 2);
    svo.setVoxel({ 1, 0, 1 }, 2);

    REQUIRE(svo.getVoxel({ 0, 0, 0 }) == 1);
}

TEST_CASE("SVO Minimal Dimension", "[SVO]")
{
    SVO<int> svo(1, 100);
    REQUIRE(svo.getVoxel({ 0, 0, 0 }) == 100);

    svo.setVoxel({ 0, 0, 0 }, 200);
    REQUIRE(svo.getVoxel({ 0, 0, 0 }) == 200);
}

TEST_CASE("SVO Type Reporting", "[SVO]")
{
    SVO<glm::vec3> svo(4, glm::vec3(0));
    std::string    type = svo.getType();
    REQUIRE(type == typeid(glm::vec3).name());
}

TEST_CASE("SVO Coarse Information Mean Strategy", "[svo][queries]")
{
    SVO<float> svo(4, 0.0f, MeanMergeStrategy<float>::instance());
    svo.setVoxel({ 0, 0, 0 }, 1.0f);

    REQUIRE(svo.getVoxel({ 0, 0, 0 }, 2) == 1.0f);
    REQUIRE(svo.getVoxel({ 0, 0, 0 }, 1) == 0.125f);
    REQUIRE(svo.getVoxel({ 0, 0, 0 }, 0) == 0.015625f);
}

TEST_CASE("SVO Coarse Information Most Prevalent Strategy", "[svo][queries]")
{
    SVO<int> svo(2, 0, MostPrevalentMergeStrategy<int>::instance());

    svo.setVoxel({ 0, 0, 0 }, 1);
    svo.setVoxel({ 1, 0, 0 }, 1);
    svo.setVoxel({ 0, 1, 0 }, 1);
    svo.setVoxel({ 1, 1, 0 }, 1);
    svo.setVoxel({ 0, 0, 1 }, 1);

    svo.setVoxel({ 1, 1, 1 }, 2);
    svo.setVoxel({ 0, 1, 1 }, 2);
    svo.setVoxel({ 1, 0, 1 }, 2);

    REQUIRE(svo.getVoxel({ 1, 1, 1 }, 1) == 2);
    REQUIRE(svo.getVoxel({ 1, 1, 1 }, 0) == 1);
}

TEST_CASE("SVO isHomogenous: Empty/Default Tree", "[SVO]")
{
    SVO<int> svo(8, 0);
    CHECK(svo.isHomogenous(glm::ivec3(0, 0, 0), glm::ivec3(8, 8, 8)) == true);
    CHECK(svo.isHomogenous(glm::ivec3(2, 2, 2), glm::ivec3(2, 2, 2)) == true);
}

TEST_CASE("SVO isHomogenous: Single Voxel Change", "[SVO]")
{
    SVO<int> svo(8, 0);
    svo.setVoxel(glm::ivec3(0, 0, 0), 1);
    CHECK(svo.isHomogenous(glm::ivec3(0, 0, 0), glm::ivec3(8, 8, 8)) == false);
    CHECK(svo.isHomogenous(glm::ivec3(1, 0, 0), glm::ivec3(7, 8, 8)) == true);
    CHECK(svo.isHomogenous(glm::ivec3(0, 0, 0), glm::ivec3(1, 1, 1)) == true);
}

TEST_CASE("SVO isHomogenous: Quadrant Testing", "[SVO]")
{
    SVO<int> svo(4, 0);
    for (int x = 0; x < 2; ++x)
        for (int y = 0; y < 2; ++y)
            for (int z = 0; z < 2; ++z)
                svo.setVoxel({ x, y, z }, 5);
    CHECK(svo.isHomogenous(glm::ivec3(0, 0, 0), glm::ivec3(2, 2, 2)) == true);
    CHECK(svo.isHomogenous(glm::ivec3(2, 2, 2), glm::ivec3(2, 2, 2)) == true);
    CHECK(svo.isHomogenous(glm::ivec3(1, 1, 1), glm::ivec3(2, 2, 2)) == false);
}

TEST_CASE("SVO isHomogenous: Boundary and Zero Size", "[SVO]")
{
    SVO<int> svo(16, 10);
    CHECK(svo.isHomogenous(glm::ivec3(0, 0, 0), glm::ivec3(0, 0, 0)) == true);
    CHECK(svo.isHomogenous(glm::ivec3(15, 15, 15), glm::ivec3(1, 1, 1)) == true);
}

TEST_CASE("SVO isHomogenous: Deep Tree Variations", "[SVO]")
{
    SVO<int> svo(128, 0);

    svo.setVoxel({ 64, 64, 64 }, 99);
    svo.setVoxel({ 65, 64, 64 }, 99);

    CHECK(svo.isHomogenous(glm::ivec3(64, 64, 64), glm::ivec3(2, 1, 1)) == true);
    CHECK(svo.isHomogenous(glm::ivec3(63, 64, 64), glm::ivec3(2, 1, 1)) == false);
}

TEST_CASE("get3DArray: Buffer Management", "[SVO][get3DArray]")
{
    SVO<int>         svo(8, 0);
    std::vector<int> buffer;
    glm::ivec3       size(2, 2, 2);

    svo.get3DArray(buffer, { 0, 0, 0 }, size);
    REQUIRE(buffer.size() == 8);

    int* originalDataPtr = buffer.data();
    svo.get3DArray(buffer, { 0, 0, 0 }, size);
    REQUIRE(buffer.data() == originalDataPtr);
}

TEST_CASE("get3DArray: Z-Major Indexing and Content", "[SVO][get3DArray]")
{
    SVO<int> svo(4, 0);

    svo.setVoxel({ 0, 0, 0 }, 1);
    svo.setVoxel({ 1, 0, 0 }, 2);
    svo.setVoxel({ 0, 1, 0 }, 3);
    svo.setVoxel({ 0, 0, 1 }, 4);

    std::vector<int> buffer;
    glm::ivec3       start(0, 0, 0);
    glm::ivec3       size(2, 2, 2);
    svo.get3DArray(buffer, start, size);

    REQUIRE(buffer[0] == 1);
    REQUIRE(buffer[1] == 2);
    REQUIRE(buffer[2] == 3);
    REQUIRE(buffer[4] == 4);
}

TEST_CASE("get3DArray: Partial Sub-Region Extraction", "[SVO][get3DArray]")
{
    SVO<int> svo(8, 10);

    for (int z = 2; z < 4; ++z)
        for (int y = 2; y < 4; ++y)
            for (int x = 2; x < 4; ++x)
                svo.setVoxel({ x, y, z }, 99);

    std::vector<int> buffer;
    svo.get3DArray(buffer, { 1, 2, 2 }, { 2, 1, 1 });

    REQUIRE(buffer.size() == 2);
    REQUIRE(buffer[0] == 10);
    REQUIRE(buffer[1] == 99);
}

TEST_CASE("get3DArray: LOD sampling at specific depth", "[SVO][get3DArray]")
{
    SVO<float> svo(4, 0.0f, MeanMergeStrategy<float>::instance());

    for (int z = 0; z < 2; ++z)
        for (int y = 0; y < 2; ++y)
            for (int x = 0; x < 2; ++x)
                svo.setVoxel({ x, y, z }, 1.0f);

    std::vector<float> buffer;
    svo.get3DArray(buffer, { 0, 0, 0 }, { 4, 4, 4 }, 1);

    REQUIRE(buffer.size() == 8);
    REQUIRE(buffer[0] == Catch::Approx(1.0f));
    REQUIRE(buffer[1] == Catch::Approx(0.0f));
}

TEST_CASE("SVO FreeList Recycling", "[SVO][FreeList]")
{
    SVO<int> svo(8, 0);

    svo.setVoxel({ 0, 0, 0 }, 10);
    REQUIRE(svo.getVoxel({ 0, 0, 0 }) == 10);
    size_t sizeAfterFirst = svo.getRaw().size();
    REQUIRE(sizeAfterFirst > 1);

    svo.setVoxel({ 0, 0, 0 }, 0);
    REQUIRE(svo.getVoxel({ 0, 0, 0 }) == 0);

    svo.setVoxel({ 0, 0, 0 }, 20);
    REQUIRE(svo.getVoxel({ 0, 0, 0 }) == 20);
    CHECK(svo.getRaw().size() <= sizeAfterFirst);
}

TEST_CASE("SVO Traverse and TraverseLeaves", "[SVO][Traverse]")
{
    SVO<int> svo(4, 0);
    svo.setVoxel({ 0, 0, 0 }, 5);
    svo.setVoxel({ 3, 3, 3 }, 9);

    int      totalLeaves = 0;
    uint32_t totalVolume = 0;

    svo.traverseLeaves(
      [&](const SVONode<int>& node, const glm::ivec3& offset, uint32_t nodeDim, int depth)
      {
          totalLeaves++;
          totalVolume += nodeDim * nodeDim * nodeDim;
          if (offset == glm::ivec3(0, 0, 0) && nodeDim == 1)
          {
              CHECK(node.value == 5);
          }
          if (offset == glm::ivec3(3, 3, 3) && nodeDim == 1)
          {
              CHECK(node.value == 9);
          }
      });

    CHECK(totalLeaves > 0);
    CHECK(totalVolume == 4 * 4 * 4);

    int visitedCount = 0;
    svo.traverse(
      [&](const SVONode<int>& node, const glm::ivec3& offset, uint32_t nodeDim, int depth, bool isLeaf) -> bool
      {
          visitedCount++;
          if (offset.x >= 2 || offset.y >= 2 || offset.z >= 2)
              return false;
          return true;
      });

    CHECK(visitedCount > 0);
}

TEST_CASE("SVO getSubSVO: Homogeneous extraction", "[SVO][SubSVO]")
{
    SVO<int> svo(8, 0);
    auto     sub0 = svo.getSubSVO({ 0, 0, 0 }, 4);
    REQUIRE(sub0 != nullptr);
    CHECK(sub0->getDimension() == 4);
    CHECK(sub0->getRaw().size() == 1);
    CHECK(sub0->getVoxel({ 0, 0, 0 }) == 0);
    CHECK(sub0->getVoxel({ 3, 3, 3 }) == 0);

    auto sub1 = svo.getSubSVO({ 4, 4, 4 }, 4);
    REQUIRE(sub1 != nullptr);
    CHECK(sub1->getDimension() == 4);
    CHECK(sub1->getRaw().size() == 1);
    CHECK(sub1->getVoxel({ 0, 0, 0 }) == 0);
}

TEST_CASE("SVO getSubSVO: Heterogeneous extraction", "[SVO][SubSVO]")
{
    SVO<int> svo(8, 0);
    svo.setVoxel({ 4, 1, 2 }, 42);
    svo.setVoxel({ 5, 0, 0 }, 99);

    auto sub = svo.getSubSVO({ 4, 0, 0 }, 4);
    REQUIRE(sub != nullptr);
    CHECK(sub->getDimension() == 4);
    CHECK(sub->getVoxel({ 0, 1, 2 }) == 42);
    CHECK(sub->getVoxel({ 1, 0, 0 }) == 99);
    CHECK(sub->getVoxel({ 0, 0, 0 }) == 0);
    CHECK(sub->getVoxel({ 3, 3, 3 }) == 0);

    auto emptySub = svo.getSubSVO({ 0, 0, 0 }, 4);
    REQUIRE(emptySub != nullptr);
    CHECK(emptySub->getRaw().size() == 1);
    CHECK(emptySub->getVoxel({ 0, 0, 0 }) == 0);
}

TEST_CASE("SVO setSubSVO: Grafting into empty tree", "[SVO][SubSVO]")
{
    SVO<int> svo(16, 0);

    SVO<int> chunk(4, 0);
    chunk.setVoxel({ 0, 0, 0 }, 10);
    chunk.setVoxel({ 3, 3, 3 }, 20);

    svo.setSubSVO({ 8, 4, 12 }, chunk);

    CHECK(svo.getVoxel({ 8, 4, 12 }) == 10);
    CHECK(svo.getVoxel({ 11, 7, 15 }) == 20);
    CHECK(svo.getVoxel({ 8, 4, 13 }) == 0);
    CHECK(svo.getVoxel({ 0, 0, 0 }) == 0);
    CHECK(svo.getVoxel({ 15, 15, 15 }) == 0);
}

TEST_CASE("SVO setSubSVO: Overwrite existing detailed subtree", "[SVO][SubSVO]")
{
    SVO<int> svo(8, 0);
    svo.setVoxel({ 0, 0, 0 }, 1);
    svo.setVoxel({ 1, 1, 1 }, 2);
    svo.setVoxel({ 4, 4, 4 }, 99);

    SVO<int> solidChunk(4, 77);
    svo.setSubSVO({ 0, 0, 0 }, solidChunk);

    CHECK(svo.getVoxel({ 0, 0, 0 }) == 77);
    CHECK(svo.getVoxel({ 1, 1, 1 }) == 77);
    CHECK(svo.getVoxel({ 3, 3, 3 }) == 77);
    CHECK(svo.getVoxel({ 4, 4, 4 }) == 99);
}

TEST_CASE("SVO setSubSVO: Homogeneous Collapse to Root", "[SVO][SubSVO]")
{
    SVO<int> svo(8, 0);
    SVO<int> chunk(4, 5);

    for (int z = 0; z < 8; z += 4)
    {
        for (int y = 0; y < 8; y += 4)
        {
            for (int x = 0; x < 8; x += 4)
            {
                svo.setSubSVO({ x, y, z }, chunk);
            }
        }
    }

    CHECK(svo.getRaw().size() == 1);
    CHECK(svo.getVoxel({ 0, 0, 0 }) == 5);
    CHECK(svo.getVoxel({ 7, 7, 7 }) == 5);
}

TEST_CASE("SVO setSubSVO / getSubSVO Roundtrip", "[SVO][SubSVO]")
{
    SVO<int> svoA(16, 0);
    svoA.setVoxel({ 8, 2, 9 }, 123);
    svoA.setVoxel({ 10, 4, 11 }, 456);

    auto sub = svoA.getSubSVO({ 8, 0, 8 }, 8);
    REQUIRE(sub != nullptr);
    CHECK(sub->getVoxel({ 0, 2, 1 }) == 123);
    CHECK(sub->getVoxel({ 2, 4, 3 }) == 456);

    SVO<int> svoB(16, 0);
    svoB.setSubSVO({ 0, 8, 8 }, sub);

    CHECK(svoB.getVoxel({ 0, 10, 9 }) == 123);
    CHECK(svoB.getVoxel({ 2, 12, 11 }) == 456);
    CHECK(svoB.getVoxel({ 0, 0, 0 }) == 0);
}

TEST_CASE("SVO setSubSVO: LOD Merging and Averaging", "[SVO][SubSVO]")
{
    SVO<float> svo(4, 0.0f, MeanMergeStrategy<float>::instance());
    SVO<float> chunk(2, 8.0f, MeanMergeStrategy<float>::instance());

    svo.setSubSVO({ 0, 0, 0 }, chunk);

    CHECK(svo.getVoxel({ 0, 0, 0 }, 0) == Catch::Approx(1.0f));
    CHECK(svo.getVoxel({ 0, 0, 0 }, 1) == Catch::Approx(8.0f));
    CHECK(svo.getVoxel({ 2, 2, 2 }, 1) == Catch::Approx(0.0f));
}

TEST_CASE("SVO setSubSVO: Same dimension full replacement", "[SVO][SubSVO]")
{
    SVO<int> svoA(8, 1);
    svoA.setVoxel({ 2, 2, 2 }, 10);

    SVO<int> svoB(8, 2);
    svoB.setVoxel({ 4, 4, 4 }, 20);

    svoA.setSubSVO({ 0, 0, 0 }, svoB);

    CHECK(svoA.getVoxel({ 2, 2, 2 }) == 2);
    CHECK(svoA.getVoxel({ 4, 4, 4 }) == 20);
}

TEST_CASE("SVO Merge Strategy: Single Strategy Enforcement and Query", "[SVO][Strategy]")
{
    SVO<int> svoDefault(8, 0);
    CHECK(&svoDefault.getMergeStrategy() != nullptr);

    SVO<int> svoMean(8, 0, MeanMergeStrategy<int>::instance());
    SVO<int> svoPrevalent(8, 0, MostPrevalentMergeStrategy<int>::instance());

    CHECK(&svoMean.getMergeStrategy() == &MeanMergeStrategy<int>::instance());
    CHECK(&svoPrevalent.getMergeStrategy() == &MostPrevalentMergeStrategy<int>::instance());
    CHECK(&svoMean.getMergeStrategy() != &svoPrevalent.getMergeStrategy());

    auto subFromMean = svoMean.getSubSVO({ 0, 0, 0 }, 4);
    REQUIRE(subFromMean != nullptr);
    CHECK(&subFromMean->getMergeStrategy() == &svoMean.getMergeStrategy());
}

TEST_CASE("SVO Uniform Grid Constructor: Homogeneous", "[SVO][UniformGrid]")
{
    uint32_t         dimension = 4;
    std::vector<int> grid(dimension * dimension * dimension, 42);
    SVO<int>         svo(grid, dimension);

    CHECK(svo.getDimension() == dimension);
    CHECK(svo.getVoxel({ 0, 0, 0 }) == 42);
    CHECK(svo.getVoxel({ 3, 3, 3 }) == 42);
    CHECK(svo.getRaw().size() == 1);
}

TEST_CASE("SVO Uniform Grid Constructor: Non-Homogeneous", "[SVO][UniformGrid]")
{
    uint32_t         dimension = 4;
    std::vector<int> grid(dimension * dimension * dimension, 0);
    grid[0 + 0 * 4 + 0 * 16] = 10;
    grid[1 + 2 * 4 + 3 * 16] = 20;

    SVO<int> svo(grid, dimension);

    CHECK(svo.getDimension() == dimension);
    CHECK(svo.getVoxel({ 0, 0, 0 }) == 10);
    CHECK(svo.getVoxel({ 1, 2, 3 }) == 20);
    CHECK(svo.getVoxel({ 0, 0, 1 }) == 0);
    CHECK(svo.getVoxel({ 3, 3, 3 }) == 0);

    for (size_t z = 0; z < dimension; z++)
    {
        for (size_t y = 0; y < dimension; y++)
        {
            for (size_t x = 0; x < dimension; x++)
            {
                int expected = grid[x + y * dimension + z * dimension * dimension];
                CHECK(svo.getVoxel(glm::ivec3(static_cast<int>(x), static_cast<int>(y), static_cast<int>(z))) == expected);
            }
        }
    }
}

TEST_CASE("SVO Uniform Grid Constructor: Dimension 1", "[SVO][UniformGrid]")
{
    std::vector<int> grid = { 99 };
    SVO<int>         svo(grid, 1);

    CHECK(svo.getDimension() == 1);
    CHECK(svo.getVoxel({ 0, 0, 0 }) == 99);
}

TEST_CASE("SVO Uniform Grid Constructor: Mean Strategy", "[SVO][UniformGrid]")
{
    uint32_t           dimension = 2;
    std::vector<float> grid(8, 0.0f);
    grid[0] = 8.0f;

    SVO<float> svo(grid, dimension, MeanMergeStrategy<float>::instance());

    CHECK(svo.getVoxel({ 0, 0, 0 }) == Catch::Approx(8.0f));
    CHECK(svo.getVoxel({ 1, 0, 0 }) == Catch::Approx(0.0f));
    CHECK(svo.getVoxel({ 0, 0, 0 }, 0) == Catch::Approx(1.0f));
}
#endif

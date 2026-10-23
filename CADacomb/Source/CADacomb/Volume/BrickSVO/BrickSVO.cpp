#include "BrickSVO.h"

namespace CADacomb
{
}

#ifdef ISTESTPROJECT
#include "CADacomb/Volume/MergeStrategy/MostPrevalentExceptMergeStrategy.h"
#include "CADacomb/Volume/MergeStrategy/MostPrevalentMergeStrategy.h"
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

namespace CADacomb
{
    TEST_CASE("BrickSVO Initial State", "[SVO]")
    {
        BrickSVO<int, 8> svo(128, 0);

        REQUIRE(svo.getDimension() == 128);
        REQUIRE(svo.getNodes().size() == 1);
        REQUIRE(svo.getBricks().size() == 0);

        REQUIRE(svo.getVoxel({ 0, 0, 0 }) == 0);
        REQUIRE(svo.getVoxel({ 64, 64, 64 }) == 0);
        REQUIRE(svo.getVoxel({ 127, 127, 127 }) == 0);
        REQUIRE(svo.getVoxel({ -1, 0, 0 }) == 0);
    }

    TEST_CASE("BrickSVO Single Voxel Modification", "[SVO]")
    {
        BrickSVO<int, 8> svo(16, 0);

        svo.setVoxel({ 4, 4, 4 }, 42);

        REQUIRE(svo.getNodes()[0].childPointer != 0xFFFFFFFF);

        REQUIRE(svo.getVoxel({ 4, 4, 4 }) == 42);
        REQUIRE(svo.getVoxel({ 5, 4, 4 }) == 0);
        REQUIRE(svo.getVoxel({ 12, 12, 12 }) == 0);
    }

    TEST_CASE("BrickSVO Dense Block Ingestion (16x16x16)", "[SVO]")
    {
        BrickSVO<int, 8> svo(16, 0);

        std::vector<int> denseData(16 * 16 * 16);
        for (int i = 0; i < 16 * 16 * 16; ++i)
        {
            denseData[i] = i + 1;
        }

        svo.set({ 0, 0, 0 }, { 16, 16, 16 }, denseData);

        REQUIRE(svo.getVoxel({ 0, 0, 0 }) == 1);
        REQUIRE(svo.getVoxel({ 15, 15, 15 }) == 4096);
        REQUIRE(svo.getVoxel({ 7, 7, 7 }) == (7 * 256 + 7 * 16 + 7 + 1));
    }

    TEST_CASE("BrickSVO Partial Boundary Ingestion", "[SVO]")
    {
        BrickSVO<int, 8> svo(16, 0);

        std::vector<int> seamData(4 * 4 * 4, 99);
        svo.set({ 6, 6, 6 }, { 4, 4, 4 }, seamData);

        REQUIRE(svo.getVoxel({ 5, 5, 5 }) == 0);
        REQUIRE(svo.getVoxel({ 6, 6, 6 }) == 99);
        REQUIRE(svo.getVoxel({ 7, 7, 7 }) == 99);
        REQUIRE(svo.getVoxel({ 8, 8, 8 }) == 99);
        REQUIRE(svo.getVoxel({ 9, 9, 9 }) == 99);
        REQUIRE(svo.getVoxel({ 10, 10, 10 }) == 0);
    }

    TEST_CASE("BrickSVO Pruning & Optimization Cleanup", "[SVO]")
    {
        BrickSVO<int, 8> svo(32, 0, 0);

        std::vector<int> uniformBrick(8 * 8 * 8, 77);
        svo.set({ 0, 0, 0 }, { 8, 8, 8 }, uniformBrick);

        REQUIRE(svo.getVoxel({ 0, 0, 0 }) == 77);

        svo.cleanup({ 0, 0, 0 }, { 8, 8, 8 });

        REQUIRE(svo.getVoxel({ 0, 0, 0 }) == 77);
        REQUIRE(svo.getVoxel({ 7, 7, 7 }) == 77);

        uint32_t depth1ChildPtr = svo.getNodes()[0].childPointer;
        REQUIRE(depth1ChildPtr != 0xFFFFFFFF);

        uint32_t depth2ChildPtr = svo.getNodes()[depth1ChildPtr].childPointer;
        REQUIRE(depth2ChildPtr != 0xFFFFFFFF);

        auto leafNode = svo.getNodes()[depth2ChildPtr + 0];
        REQUIRE(leafNode.value == 77);
        REQUIRE(leafNode.brickPointer == 0xFFFFFFFF);
    }

    enum class TestMaterialType : uint8_t
    {
        Air   = 0,
        Stone = 1,
        Dirt  = 2,
        Gold  = 3
    };

    TEST_CASE("BrickSVO MostPrevalentMergeStrategy Voxel Selection", "[SVO]")
    {
        BrickSVO<TestMaterialType, 8> svo(32, TestMaterialType::Air, MostPrevalentMergeStrategy<TestMaterialType>::instance(), 0);

        glm::ivec3                    leafDim(8, 8, 8);
        std::vector<TestMaterialType> mixedData(8 * 8 * 8, TestMaterialType::Stone);

        for (int z = 0; z < 2; ++z)
        {
            for (int y = 0; y < 2; ++y)
            {
                for (int x = 0; x < 2; ++x)
                {
                    int flatIdx = (z * 64) + (y * 8) + x;

                    if (z * 4 + y * 2 + x < 5)
                    {
                        mixedData[flatIdx] = TestMaterialType::Gold;
                    }
                    else
                    {
                        mixedData[flatIdx] = TestMaterialType::Dirt;
                    }
                }
            }
        }

        svo.set({ 0, 0, 0 }, leafDim, mixedData);
        svo.cleanup({ 0, 0, 0 }, { 8, 8, 8 });

        uint32_t rootChildPtr = svo.getNodes()[0].childPointer;
        REQUIRE(rootChildPtr != 0xFFFFFFFF);

        uint32_t depth1NodeIdx = rootChildPtr + 0;
        uint32_t lodBrickIdx   = svo.getNodes()[depth1NodeIdx].brickPointer;
        REQUIRE(lodBrickIdx != 0xFFFFFFFF);

        TestMaterialType lodVoxelResult = svo.getBricks()[lodBrickIdx].data[0];
        REQUIRE(lodVoxelResult == TestMaterialType::Gold);
    }

    TEST_CASE("BrickSVO MostPrevalentMergeStrategy Tie Breaker", "[SVO]")
    {
        BrickSVO<int, 8> svo(32, 0, MostPrevalentExceptMergeStrategy<int, 0>::instance(), 0);

        glm::ivec3       leafDim(8, 8, 8);
        std::vector<int> tieData(8 * 8 * 8, 0);

        for (int i = 0; i < 8; ++i)
        {
            tieData[i] = (i < 4) ? 10 : 20;
        }

        svo.set({ 0, 0, 0 }, leafDim, tieData);
        svo.cleanup({ 0, 0, 0 }, { 8, 8, 8 });

        uint32_t depth1NodeIdx = svo.getNodes()[0].childPointer + 0;
        uint32_t lodBrickIdx   = svo.getNodes()[depth1NodeIdx].brickPointer;
        REQUIRE(0xFFFFFFFF != lodBrickIdx);
        const auto& bData          = svo.getBricks()[lodBrickIdx].data;
        int         lodVoxelResult = bData[0];
        REQUIRE(lodVoxelResult == 10);
    }

    TEST_CASE("BrickSVO minLODDepth Limit Enforcement", "[SVO]")
    {
        uint32_t minLODDepth = 1;

        BrickSVO<int, 8> svo(16, 0, MostPrevalentMergeStrategy<int>::instance(), minLODDepth);

        glm::ivec3       leafDim(7, 7, 7);
        std::vector<int> detailedData(7 * 7 * 7, 42);

        svo.set({ 0, 0, 0 }, leafDim, detailedData);
        svo.cleanup({ 0, 0, 0 }, { 16, 16, 16 });

        uint32_t depth1NodeIdx = svo.getNodes()[0].childPointer + 0;
        uint32_t lod1BrickIdx  = svo.getNodes()[depth1NodeIdx].brickPointer;

        REQUIRE(lod1BrickIdx != 0xFFFFFFFF);
        REQUIRE(svo.getBricks()[lod1BrickIdx].data[0] == 42);

        uint32_t rootBrickIdx = svo.getNodes()[0].brickPointer;
        REQUIRE(rootBrickIdx == 0xFFFFFFFF);
    }

    struct DummyMerge : public MergeStrategy<int>
    {
        DummyMerge() {}

        virtual ~DummyMerge() {}

        int merge(const std::array<int, 8>& inputs) const override
        {
            return inputs[0];
        }
    };

    TEST_CASE("BrickSVO Padded: Single Voxel Set and Get")
    {
        DummyMerge             dummyMerge;
        BrickSVO<int, 8, true> svo(16, 0, dummyMerge, 1);

        glm::ivec3 targetPos(3, 4, 5);
        svo.set(targetPos, glm::ivec3(1, 1, 1), { 42 });

        REQUIRE(svo.getVoxel(targetPos) == 42);
        REQUIRE(svo.getVoxel(glm::ivec3(0, 0, 0)) == 0);
    }

    TEST_CASE("BrickSVO Padded: Full Brick Block Write")
    {
        constexpr unsigned short    B_SIZE = 8;
        DummyMerge                  dummyMerge;
        BrickSVO<int, B_SIZE, true> svo(16, 0, dummyMerge, 1);

        glm::ivec3 start(0, 0, 0);
        glm::ivec3 dim(B_SIZE, B_SIZE, B_SIZE);

        std::vector<int> inputData(B_SIZE * B_SIZE * B_SIZE);
        for (int i = 0; i < inputData.size(); ++i)
        {
            inputData[i] = i + 1;
        }

        svo.set(start, dim, inputData);

        REQUIRE(svo.getVoxel(glm::ivec3(0, 0, 0)) == 1);
        REQUIRE(svo.getVoxel(glm::ivec3(7, 0, 0)) == 8);
        REQUIRE(svo.getVoxel(glm::ivec3(7, 7, 7)) == 512);
    }

    TEST_CASE("BrickSVO Padded: Raw Brick Storage and Layout Validation")
    {
        constexpr unsigned short    B_SIZE = 8;
        DummyMerge                  dummyMerge;
        BrickSVO<int, B_SIZE, true> svo(16, 0, dummyMerge, 1);

        svo.set(glm::ivec3(0, 0, 0), glm::ivec3(1, 1, 1), { 99 });

        const auto& bricks = svo.getBricks();
        REQUIRE(bricks.size() == 1);

        constexpr size_t expectedStorageSize = (B_SIZE + 2) * (B_SIZE + 2) * (B_SIZE + 2);
        REQUIRE(sizeof(bricks[0].data) / sizeof(int) == expectedStorageSize);

        constexpr size_t allocatedSize   = B_SIZE + 2;
        size_t           expectedFlatIdx = 1 + (1 * allocatedSize) + (1 * allocatedSize * allocatedSize);

        REQUIRE(bricks[0].data[expectedFlatIdx] == 99);
    }

    TEST_CASE("BrickSVO Padded: Boundary Overlap and Strided Read")
    {
        DummyMerge             dummyMerge;
        BrickSVO<int, 8, true> svo(16, 0, dummyMerge, 1);

        glm::ivec3       start(7, 0, 0);
        glm::ivec3       dim(2, 1, 1);
        std::vector<int> data = { 101, 102 };

        svo.set(start, dim, data);

        REQUIRE(svo.getVoxel(glm::ivec3(7, 0, 0)) == 101);
        REQUIRE(svo.getVoxel(glm::ivec3(8, 0, 0)) == 102);
    }

    TEST_CASE("Brick - Unpadded size verification")
    {
        Brick<uint8_t, 8, false> unpaddedBrick;
        REQUIRE(sizeof(unpaddedBrick.data) == 512);
    }

    TEST_CASE("Brick - Padded size verification")
    {
        Brick<uint8_t, 8, true> paddedBrick;
        REQUIRE(sizeof(paddedBrick.data) == 1000);
    }

    TEST_CASE("BrickSVO - Initialization defaults with padding disabled")
    {
        uint32_t                    dimension    = 64;
        uint8_t                     defaultValue = 0;
        BrickSVO<uint8_t, 8, false> svo(dimension, defaultValue, 2);

        REQUIRE(svo.getDimension() == dimension);
        REQUIRE(svo.getBricks().empty());
        REQUIRE_FALSE(svo.getNodes().empty());
    }

    TEST_CASE("BrickSVO - Initialization defaults with padding enabled")
    {
        uint32_t                   dimension    = 64;
        uint8_t                    defaultValue = 0;
        BrickSVO<uint8_t, 8, true> svo(dimension, defaultValue, 2);

        REQUIRE(svo.getDimension() == dimension);
        REQUIRE(svo.getBricks().empty());
    }

    TEST_CASE("BrickSVO - Set and Get Voxel with unpadded configuration")
    {
        BrickSVO<uint8_t, 8, false> svo(64, 0, 2);
        glm::ivec3                  targetPos(4, 4, 4);
        uint8_t                     expectedValue = 42;

        svo.setVoxel(targetPos, expectedValue);

        REQUIRE(svo.getVoxel(targetPos) == expectedValue);
    }

    TEST_CASE("BrickSVO - Set and Get Voxel with padded configuration")
    {
        BrickSVO<uint8_t, 8, true> svo(64, 0, 2);
        glm::ivec3                 targetPos(12, 3, 45);
        uint8_t                    expectedValue = 99;

        svo.setVoxel(targetPos, expectedValue);

        REQUIRE(svo.getVoxel(targetPos) == expectedValue);
    }

    TEST_CASE("BrickSVO - Bulk data array injection (Unpadded)")
    {
        BrickSVO<uint8_t, 8, false> svo(16, 0, 1);
        glm::ivec3                  start(0, 0, 0);
        glm::ivec3                  size(8, 8, 8);

        std::vector<uint8_t> bulkData(512, 7);

        svo.set(start, size, bulkData);

        REQUIRE(svo.getVoxel(glm::ivec3(0, 0, 0)) == 7);
        REQUIRE(svo.getVoxel(glm::ivec3(7, 7, 7)) == 7);
    }

    TEST_CASE("BrickSVO - Non-default brickSize initialization and allocation", "[svo][bricksize]")
    {
        {
            BrickSVO<float, 4, false> svo(64, 0.0f, 2);

            REQUIRE(svo.getDimension() == 64);
            REQUIRE(svo.getNodes().size() == 1);
            REQUIRE(svo.getBricks().size() == 0);

            REQUIRE(sizeof(Brick<float, 4, false>::data) / sizeof(float) == 64);
        }

        {
            BrickSVO<int, 16, true> svo(128, -1, 2);

            REQUIRE(svo.getDimension() == 128);

            REQUIRE(sizeof(Brick<int, 16, true>::data) / sizeof(int) == 5832);
        }
    }

    TEST_CASE("BrickSVO - Voxel set and get with brickSize = 4", "[svo][bricksize]")
    {
        BrickSVO<int, 4, false> svo(16, 0, 1);

        svo.setVoxel(glm::ivec3(0, 0, 0), 42);
        svo.setVoxel(glm::ivec3(3, 3, 3), 24);
        svo.setVoxel(glm::ivec3(4, 0, 0), 99);
        svo.setVoxel(glm::ivec3(15, 15, 15), 7);

        REQUIRE(svo.getVoxel(glm::ivec3(0, 0, 0)) == 42);
        REQUIRE(svo.getVoxel(glm::ivec3(3, 3, 3)) == 24);
        REQUIRE(svo.getVoxel(glm::ivec3(4, 0, 0)) == 99);
        REQUIRE(svo.getVoxel(glm::ivec3(15, 15, 15)) == 7);

        REQUIRE(svo.getVoxel(glm::ivec3(1, 1, 1)) == 0);
        REQUIRE(svo.getVoxel(glm::ivec3(5, 5, 5)) == 0);
    }

    TEST_CASE("BrickSVO - Bulk data set and cleanup with brickSize = 16", "[svo][bricksize]")
    {
        const uint32_t                bSize = 16;
        BrickSVO<float, bSize, false> svo(bSize, 0.0f, 0);

        std::vector<float> bulkData(bSize * bSize * bSize, 1.5f);

        svo.set(glm::ivec3(0, 0, 0), glm::ivec3(bSize, bSize, bSize), bulkData);

        REQUIRE(svo.getVoxel(glm::ivec3(0, 0, 0)) == 1.5f);
        REQUIRE(svo.getVoxel(glm::ivec3(bSize - 1, bSize - 1, bSize - 1)) == 1.5f);

        svo.cleanup(glm::ivec3(0, 0, 0), glm::ivec3(bSize, bSize, bSize));

        REQUIRE(svo.getVoxel(glm::ivec3(5, 5, 5)) == 1.5f);
    }

    TEST_CASE("BrickSVO - Strict padding verification for all bricks", "[svo][padding]")
    {
        const unsigned short BRICK_SIZE  = 4;
        const unsigned short PADDED_SIZE = BRICK_SIZE + 2;
        const uint32_t       DIMENSION   = 16;

        BrickSVO<int, BRICK_SIZE, true> svo(DIMENSION, 0, 0);

        std::vector<int> bulkData(DIMENSION * DIMENSION * DIMENSION);
        for (int z = 0; z < DIMENSION; ++z)
        {
            for (int y = 0; y < DIMENSION; ++y)
            {
                for (int x = 0; x < DIMENSION; ++x)
                {
                    size_t idx    = x + (y * DIMENSION) + (z * DIMENSION * DIMENSION);
                    bulkData[idx] = static_cast<int>(idx + 1);
                }
            }
        }
        svo.set(glm::ivec3(0, 0, 0), glm::ivec3(DIMENSION, DIMENSION, DIMENSION), bulkData);

        svo.cleanup(glm::ivec3(0, 0, 0), glm::ivec3(DIMENSION, DIMENSION, DIMENSION));

        const auto& allocatedBricks = svo.getBricks();
        REQUIRE_FALSE(allocatedBricks.empty());

        for (const auto& brick : allocatedBricks)
        {
            for (int z = 0; z < PADDED_SIZE; ++z)
            {
                for (int y = 0; y < PADDED_SIZE; ++y)
                {
                    for (int x = 0; x < PADDED_SIZE; ++x)
                    {
                        bool isPadding = (x == 0 || x == BRICK_SIZE + 1 || y == 0 || y == BRICK_SIZE + 1 || z == 0 || z == BRICK_SIZE + 1);

                        size_t localIdx = x + (y * PADDED_SIZE) + (z * PADDED_SIZE * PADDED_SIZE);
                        int    val      = brick.data[localIdx];

                        if (isPadding)
                        {
                            REQUIRE(val > 0);
                        }
                    }
                }
            }
        }
    }

    TEST_CASE("BrickSVO - minLODDepth respected", "[svo][minLODDepth]")
    {
        const unsigned short BRICK_SIZE = 4;
        const uint32_t       DIMENSION  = 32;
        auto                 fillUnique = [](auto& svo)
        {
            std::vector<int> bulkData(DIMENSION * DIMENSION * DIMENSION);
            for (int z = 0; z < DIMENSION; ++z)
            {
                for (int y = 0; y < DIMENSION; ++y)
                {
                    for (int x = 0; x < DIMENSION; ++x)
                    {
                        size_t idx    = x + (y * DIMENSION) + (z * DIMENSION * DIMENSION);
                        bulkData[idx] = static_cast<int>(idx + 1);
                    }
                }
            }
            svo.set(glm::ivec3(0, 0, 0), glm::ivec3(DIMENSION, DIMENSION, DIMENSION), bulkData);
            svo.cleanup(glm::ivec3(0, 0, 0), glm::ivec3(DIMENSION, DIMENSION, DIMENSION));
        };

        BrickSVO<int, BRICK_SIZE, true> min0(DIMENSION, 0, 0);
        BrickSVO<int, BRICK_SIZE, true> min1(DIMENSION, 0, 1);
        BrickSVO<int, BRICK_SIZE, true> min2(DIMENSION, 0, 2);
        BrickSVO<int, BRICK_SIZE, true> minX(DIMENSION, 0, 500);
        fillUnique(min0);
        fillUnique(min1);
        fillUnique(min2);
        fillUnique(minX);

        auto child0_depth1 = min0.getNodes()[0].childPointer;
        auto child1_depth1 = min1.getNodes()[0].childPointer;
        auto child2_depth1 = min2.getNodes()[0].childPointer;
        auto childX_depth1 = minX.getNodes()[0].childPointer;
        REQUIRE(child0_depth1 != 0xFFFFFFFF);
        REQUIRE(child1_depth1 != 0xFFFFFFFF);
        REQUIRE(child2_depth1 != 0xFFFFFFFF);
        REQUIRE(childX_depth1 != 0xFFFFFFFF);
        auto child0_depth2 = min0.getNodes()[child0_depth1].childPointer;
        auto child1_depth2 = min1.getNodes()[child1_depth1].childPointer;
        auto child2_depth2 = min2.getNodes()[child2_depth1].childPointer;
        auto childX_depth2 = min2.getNodes()[childX_depth1].childPointer;
        REQUIRE(child0_depth2 != 0xFFFFFFFF);
        REQUIRE(child1_depth2 != 0xFFFFFFFF);
        REQUIRE(child2_depth2 != 0xFFFFFFFF);
        REQUIRE(childX_depth2 != 0xFFFFFFFF);
        auto child0_depth3 = min0.getNodes()[child0_depth2].childPointer;
        auto child1_depth3 = min1.getNodes()[child1_depth2].childPointer;
        auto child2_depth3 = min2.getNodes()[child2_depth2].childPointer;
        auto childX_depth3 = min2.getNodes()[childX_depth2].childPointer;
        REQUIRE(child0_depth3 != 0xFFFFFFFF);
        REQUIRE(child1_depth3 != 0xFFFFFFFF);
        REQUIRE(child2_depth3 != 0xFFFFFFFF);
        REQUIRE(childX_depth3 != 0xFFFFFFFF);
        REQUIRE(min0.getNodes()[child0_depth3].childPointer == 0xFFFFFFFF);
        REQUIRE(min1.getNodes()[child1_depth3].childPointer == 0xFFFFFFFF);
        REQUIRE(min2.getNodes()[child2_depth3].childPointer == 0xFFFFFFFF);
        REQUIRE(minX.getNodes()[childX_depth3].childPointer == 0xFFFFFFFF);

        REQUIRE(min0.getNodes()[0].brickPointer != 0xFFFFFFFF);
        REQUIRE(min1.getNodes()[0].brickPointer == 0xFFFFFFFF);
        REQUIRE(min2.getNodes()[0].brickPointer == 0xFFFFFFFF);
        REQUIRE(minX.getNodes()[0].brickPointer == 0xFFFFFFFF);

        REQUIRE(min0.getNodes()[child0_depth1].brickPointer != 0xFFFFFFFF);
        REQUIRE(min1.getNodes()[child1_depth1].brickPointer != 0xFFFFFFFF);
        REQUIRE(min2.getNodes()[child2_depth1].brickPointer == 0xFFFFFFFF);
        REQUIRE(minX.getNodes()[childX_depth1].brickPointer == 0xFFFFFFFF);

        REQUIRE(min0.getNodes()[child0_depth2].brickPointer != 0xFFFFFFFF);
        REQUIRE(min1.getNodes()[child1_depth2].brickPointer != 0xFFFFFFFF);
        REQUIRE(min2.getNodes()[child2_depth2].brickPointer != 0xFFFFFFFF);
        REQUIRE(minX.getNodes()[childX_depth2].brickPointer == 0xFFFFFFFF);

        REQUIRE(min0.getNodes()[child0_depth3].brickPointer != 0xFFFFFFFF);
        REQUIRE(min1.getNodes()[child1_depth3].brickPointer != 0xFFFFFFFF);
        REQUIRE(min2.getNodes()[child2_depth3].brickPointer != 0xFFFFFFFF);
        REQUIRE(minX.getNodes()[childX_depth3].brickPointer != 0xFFFFFFFF);
    }
}
#endif

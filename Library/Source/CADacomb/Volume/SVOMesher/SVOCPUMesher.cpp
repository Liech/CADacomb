#include "SVOCPUMesher.h"

#include <cmath>
#include <map>
#include <set>
#include <tuple>

namespace Library::Volume
{
    SVOCPUMesher::SVOCPUMesher() {}

    SVOCPUMesher::~SVOCPUMesher() {}

    void SVOCPUMesher::generateBorderChunks(std::vector<MeshChunk>& chunks)
    {
        std::erase_if(chunks, [](const MeshChunk& c) { return c.isBorder; });

        size_t interiorCount = chunks.size();
        if (interiorCount == 0)
        {
            return;
        }

        std::map<std::tuple<int, int, int>, size_t> chunkMap;
        for (size_t i = 0; i < interiorCount; i++)
        {
            chunkMap[{ chunks[i].minBound.x, chunks[i].minBound.y, chunks[i].minBound.z }] = i;
        }

        std::set<std::tuple<int, int, int, uint8_t>> visitedBorders;

        for (size_t i = 0; i < interiorCount; i++)
        {
            glm::ivec3 cMin  = chunks[i].minBound;
            uint32_t   cSize = chunks[i].size;
            int        cLod  = chunks[i].lodLevel;
            uint32_t   cRes  = chunks[i].maxResolution;

            for (uint8_t axis = 0; axis < 3; axis++)
            {
                glm::ivec3 negBound = cMin;
                negBound[axis] -= static_cast<int>(cSize);
                auto negKey = std::make_tuple(negBound.x, negBound.y, negBound.z, axis);
                if (visitedBorders.insert(negKey).second)
                {
                    auto it = chunkMap.find({ negBound.x, negBound.y, negBound.z });
                    if (it != chunkMap.end())
                    {
                        const auto& negChunk = chunks[it->second];
                        chunks.push_back(MeshChunk::createBorder(negBound, cSize, axis, true, negChunk.lodLevel, negChunk.maxResolution, true, cLod, cRes));
                    }
                    else
                    {
                        chunks.push_back(MeshChunk::createBorder(negBound, cSize, axis, false, cLod, cRes, true, cLod, cRes));
                    }
                }

                glm::ivec3 posBound = cMin;
                auto       posKey   = std::make_tuple(posBound.x, posBound.y, posBound.z, axis);
                if (visitedBorders.insert(posKey).second)
                {
                    glm::ivec3 neighborMin = cMin;
                    neighborMin[axis] += static_cast<int>(cSize);
                    auto it = chunkMap.find({ neighborMin.x, neighborMin.y, neighborMin.z });
                    if (it != chunkMap.end())
                    {
                        const auto& posChunk = chunks[it->second];
                        chunks.push_back(MeshChunk::createBorder(posBound, cSize, axis, true, cLod, cRes, true, posChunk.lodLevel, posChunk.maxResolution));
                    }
                    else
                    {
                        chunks.push_back(MeshChunk::createBorder(posBound, cSize, axis, true, cLod, cRes, false, cLod, cRes));
                    }
                }
            }
        }
    }

    std::unique_ptr<Library::Triangulation> SVOCPUMesher::extractTriangulation(const std::vector<MeshChunk>& chunks, const glm::dvec3& worldOrigin, double voxelWorldSize)
    {
        auto result = std::make_unique<Library::Triangulation>();

        for (const auto& chunk : chunks)
        {
            glm::dvec3 chunkOrigin = worldOrigin + glm::dvec3(chunk.minBound) * voxelWorldSize;
            for (const auto& q : chunk.quads)
            {
                auto   corners = q.getCorners(chunkOrigin, voxelWorldSize);
                size_t baseIdx = result->vertices.size();

                result->vertices.push_back(corners[0]);
                result->vertices.push_back(corners[1]);
                result->vertices.push_back(corners[2]);
                result->vertices.push_back(corners[3]);

                result->indices.push_back(baseIdx + 0);
                result->indices.push_back(baseIdx + 1);
                result->indices.push_back(baseIdx + 2);

                result->indices.push_back(baseIdx + 0);
                result->indices.push_back(baseIdx + 2);
                result->indices.push_back(baseIdx + 3);
            }
        }

        return result;
    }

    std::unique_ptr<Library::Triangulation> SVOCPUMesher::extractTriangulation(const MeshChunk& chunk, const glm::dvec3& worldOrigin, double voxelWorldSize)
    {
        std::vector<MeshChunk> singleChunkList = { chunk };
        return extractTriangulation(singleChunkList, worldOrigin, voxelWorldSize);
    }
}

#ifdef ISTESTPROJECT
#include "CADacomb/Volume/SVO/Generator/SVOLandscapeGenerator.h"
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

namespace Library::Volume
{
    static const MeshChunk* findBorderChunk(const std::vector<MeshChunk>& chunks, const glm::ivec3& minBound, uint8_t axis)
    {
        for (const auto& c : chunks)
        {
            if (c.isBorder && c.borderAxis == axis && c.minBound == minBound)
            {
                return &c;
            }
        }
        return nullptr;
    }

    static size_t computeDirQuadArea(const MeshChunk& chunk, uint32_t dir)
    {
        size_t area = 0;
        for (const auto& q : chunk.quads)
        {
            if (q.getDirection() == dir)
            {
                uint32_t s = q.getScale();
                area += static_cast<size_t>(s) * static_cast<size_t>(s);
            }
        }
        return area;
    }

    TEST_CASE("SVOMesher/SingleChunkMeshing", "[SVOMesher][Chunk]")
    {
        SVO<bool> svo(32, false);

        for (int z = 4; z < 12; z++)
        {
            for (int y = 4; y < 12; y++)
            {
                for (int x = 4; x < 12; x++)
                {
                    svo.setVoxel({ x, y, z }, true);
                }
            }
        }

        MeshChunk chunk;
        chunk.minBound      = glm::ivec3(0);
        chunk.size          = 32;
        chunk.maxResolution = 32;

        SVOCPUMesher::meshChunk(svo, chunk);

        CHECK(!chunk.quads.empty());

        auto mesh = SVOCPUMesher::extractTriangulation(chunk);
        REQUIRE(mesh != nullptr);
        CHECK(!mesh->vertices.empty());
        CHECK(!mesh->indices.empty());
        CHECK(mesh->indices.size() % 6 == 0);

        auto [minB, sz] = mesh->getAABB();
        CHECK(minB.x >= 4.0 - 1e-4);
        CHECK(minB.y >= 4.0 - 1e-4);
        CHECK(minB.z >= 4.0 - 1e-4);
        CHECK(minB.x + sz.x <= 12.0 + 1e-4);
        CHECK(minB.y + sz.y <= 12.0 + 1e-4);
        CHECK(minB.z + sz.z <= 12.0 + 1e-4);
    }

    TEST_CASE("SVOMesher/WatertightBoundaryInterface", "[SVOMesher][Boundary]")
    {
        SVO<bool> svo(64, false);

        for (int z = 0; z < 16; z++)
        {
            for (int y = 0; y < 16; y++)
            {
                for (int x = 16; x < 48; x++)
                {
                    svo.setVoxel({ x, y, z }, true);
                }
            }
        }

        MeshChunk chunkA;
        chunkA.minBound      = glm::ivec3(0, 0, 0);
        chunkA.size          = 32;
        chunkA.maxResolution = 32;

        MeshChunk chunkB;
        chunkB.minBound      = glm::ivec3(32, 0, 0);
        chunkB.size          = 32;
        chunkB.maxResolution = 16;

        std::vector<MeshChunk> chunks = { chunkA, chunkB };
        SVOCPUMesher::meshChunks(svo, chunks);

        const MeshChunk* seam = findBorderChunk(chunks, glm::ivec3(0, 0, 0), 0);
        REQUIRE(seam != nullptr);
        CHECK(seam->quads.empty());

        auto unifiedMesh = SVOCPUMesher::extractTriangulation(chunks);
        REQUIRE(unifiedMesh != nullptr);
        CHECK(!unifiedMesh->vertices.empty());

        auto [minB, sz] = unifiedMesh->getAABB();
        CHECK(minB.x == Catch::Approx(16.0));
        CHECK(minB.x + sz.x == Catch::Approx(48.0));
    }

    TEST_CASE("SVOMesher/MultiChunkLandscapeTriangulation", "[SVOMesher][Landscape]")
    {
        LandscapeConfig config;
        config.seed            = 1337;
        config.octaves         = 1;
        config.baseHeightRatio = 0.5;

        auto svo = SVOLandscapeGenerator::generateSVO<bool>(64, config);
        REQUIRE(svo != nullptr);

        std::vector<MeshChunk> chunks;
        for (int z = 0; z < 64; z += 32)
        {
            for (int y = 0; y < 64; y += 32)
            {
                for (int x = 0; x < 64; x += 32)
                {
                    MeshChunk c;
                    c.minBound      = glm::ivec3(x, y, z);
                    c.size          = 32;
                    c.maxResolution = 32;
                    chunks.push_back(c);
                }
            }
        }

        SVOCPUMesher::meshChunks(*svo, chunks);

        auto mesh = SVOCPUMesher::extractTriangulation(chunks);
        REQUIRE(mesh != nullptr);
        CHECK(!mesh->vertices.empty());
        CHECK(!mesh->indices.empty());
        CHECK(mesh->indices.size() % 6 == 0);

        auto [minB, sz] = mesh->getAABB();
        CHECK(minB.x >= 0.0);
        CHECK(minB.y >= 0.0);
        CHECK(minB.z >= 0.0);
        CHECK(minB.x + sz.x <= 64.0 + 1e-4);
        CHECK(minB.y + sz.y <= 64.0 + 1e-4);
        CHECK(minB.z + sz.z <= 64.0 + 1e-4);
    }

    TEST_CASE("SVOMesher/AdaptiveLODChunkGeneration", "[SVOMesher][Adaptive]")
    {
        SVO<bool>        svo(128, false);
        SVOMesherOptions options;
        options.chunkSize       = 32;
        options.maxResolution   = 32;
        options.adaptive        = true;
        options.focalPoint      = glm::dvec3(16.0, 16.0, 16.0);
        options.lodNearDistance = 10.0;
        options.lodFarDistance  = 80.0;
        options.minLOD          = 0;
        options.maxLOD          = 2;
        options.origin          = glm::dvec3(0.0);
        options.size            = glm::dvec3(128.0);

        auto chunks = SVOCPUMesher::generateChunks(svo, options);
        REQUIRE(chunks.size() == 64);

        CHECK(chunks[0].lodLevel == 0);
        CHECK(chunks[0].getCellSize() == 1);

        const auto& farChunk = chunks.back();
        CHECK(farChunk.lodLevel > 0);
        CHECK(farChunk.getCellSize() > 1);
    }

    TEST_CASE("SVOMesher/HighLevelTriangulateAPI", "[SVOMesher][Triangulate]")
    {
        LandscapeConfig config;
        config.seed            = 42;
        config.octaves         = 1;
        config.baseHeightRatio = 0.5;

        auto svo = SVOLandscapeGenerator::generateSVO<bool>(64, config);
        REQUIRE(svo != nullptr);

        SVOMesherOptions options;
        options.chunkSize     = 32;
        options.maxResolution = 32;
        options.origin        = glm::dvec3(0.0);
        options.size          = glm::dvec3(64.0);

        auto mesh = SVOCPUMesher::triangulate(*svo, options);
        REQUIRE(mesh != nullptr);
        CHECK(!mesh->vertices.empty());
        CHECK(!mesh->indices.empty());
        CHECK(mesh->indices.size() % 6 == 0);
    }

    TEST_CASE("SVOMesher/LODDiffContinuousSolidNoInternalSeamFaces", "[SVOMesher][Watertight][Boundary]")
    {
        SVO<bool> svo(64, false);

        for (int z = 4; z < 28; z++)
        {
            for (int y = 4; y < 28; y++)
            {
                for (int x = 16; x < 48; x++)
                {
                    svo.setVoxel({ x, y, z }, true);
                }
            }
        }

        MeshChunk chunkA;
        chunkA.minBound      = glm::ivec3(0, 0, 0);
        chunkA.size          = 32;
        chunkA.maxResolution = 32;

        MeshChunk chunkB;
        chunkB.minBound      = glm::ivec3(32, 0, 0);
        chunkB.size          = 32;
        chunkB.maxResolution = 16;

        std::vector<MeshChunk> chunks = { chunkA, chunkB };
        SVOCPUMesher::meshChunks(svo, chunks);

        CHECK(chunks[0].getCellSize() == 1);
        CHECK(chunks[1].getCellSize() == 2);

        const MeshChunk* seam = findBorderChunk(chunks, glm::ivec3(0, 0, 0), 0);
        REQUIRE(seam != nullptr);
        CHECK(seam->quads.empty());
    }

    TEST_CASE("SVOMesher/LODDiffFineSolidStopsAtSeam", "[SVOMesher][Watertight][Boundary]")
    {
        SVO<bool> svo(64, false);

        for (int z = 8; z < 24; z++)
        {
            for (int y = 8; y < 24; y++)
            {
                for (int x = 16; x < 32; x++)
                {
                    svo.setVoxel({ x, y, z }, true);
                }
            }
        }

        MeshChunk chunkA;
        chunkA.minBound      = glm::ivec3(0, 0, 0);
        chunkA.size          = 32;
        chunkA.maxResolution = 32;

        MeshChunk chunkB;
        chunkB.minBound      = glm::ivec3(32, 0, 0);
        chunkB.size          = 32;
        chunkB.maxResolution = 16;

        std::vector<MeshChunk> chunks = { chunkA, chunkB };
        SVOCPUMesher::meshChunks(svo, chunks);

        const MeshChunk* seam = findBorderChunk(chunks, glm::ivec3(0, 0, 0), 0);
        REQUIRE(seam != nullptr);
        CHECK(computeDirQuadArea(*seam, 1) == 16 * 16);
        CHECK(computeDirQuadArea(*seam, 0) == 0);

        for (const auto& q : seam->quads)
        {
            CHECK(q.getDirection() == 1);
            CHECK(q.getX() + q.getScale() == 32);
        }
    }

    TEST_CASE("SVOMesher/LODDiffCoarseSolidStartsAtSeamSubdivided", "[SVOMesher][Watertight][Boundary]")
    {
        SVO<bool> svo(64, false);

        for (int z = 8; z < 24; z++)
        {
            for (int y = 8; y < 24; y++)
            {
                for (int x = 32; x < 48; x++)
                {
                    svo.setVoxel({ x, y, z }, true);
                }
            }
        }

        MeshChunk chunkA;
        chunkA.minBound      = glm::ivec3(0, 0, 0);
        chunkA.size          = 32;
        chunkA.maxResolution = 32;

        MeshChunk chunkB;
        chunkB.minBound      = glm::ivec3(32, 0, 0);
        chunkB.size          = 32;
        chunkB.maxResolution = 16;

        std::vector<MeshChunk> chunks = { chunkA, chunkB };
        SVOCPUMesher::meshChunks(svo, chunks);

        const MeshChunk* seam = findBorderChunk(chunks, glm::ivec3(0, 0, 0), 0);
        REQUIRE(seam != nullptr);
        CHECK(computeDirQuadArea(*seam, 1) == 0);
        CHECK(computeDirQuadArea(*seam, 0) == 16 * 16);

        for (const auto& q : seam->quads)
        {
            CHECK(q.getDirection() == 0);
            CHECK(q.getX() == 32);
        }
    }

    TEST_CASE("SVOMesher/LODDiffStepHeightWatertight", "[SVOMesher][Watertight][Boundary]")
    {
        SVO<bool> svo(64, false);

        for (int z = 8; z < 24; z++)
        {
            for (int y = 0; y < 10; y++)
            {
                for (int x = 16; x < 32; x++)
                {
                    svo.setVoxel({ x, y, z }, true);
                }
            }
        }

        for (int z = 8; z < 24; z++)
        {
            for (int y = 0; y < 8; y++)
            {
                for (int x = 32; x < 48; x++)
                {
                    svo.setVoxel({ x, y, z }, true);
                }
            }
        }

        MeshChunk chunkA;
        chunkA.minBound      = glm::ivec3(0, 0, 0);
        chunkA.size          = 32;
        chunkA.maxResolution = 32;

        MeshChunk chunkB;
        chunkB.minBound      = glm::ivec3(32, 0, 0);
        chunkB.size          = 32;
        chunkB.maxResolution = 16;

        std::vector<MeshChunk> chunks = { chunkA, chunkB };
        SVOCPUMesher::meshChunks(svo, chunks);

        const MeshChunk* seam = findBorderChunk(chunks, glm::ivec3(0, 0, 0), 0);
        REQUIRE(seam != nullptr);
        CHECK(computeDirQuadArea(*seam, 1) == 2 * 16);
        CHECK(computeDirQuadArea(*seam, 0) == 0);

        for (const auto& q : seam->quads)
        {
            CHECK(q.getDirection() == 1);
            CHECK(q.getX() + q.getScale() == 32);
            CHECK(q.getY() >= 8);
            CHECK(q.getY() + q.getScale() <= 10);
        }
    }

    TEST_CASE("SVOMesher/LODDiffStepHeightCoarseHigherWatertight", "[SVOMesher][Watertight][Boundary]")
    {
        SVO<bool> svo(64, false);

        for (int z = 8; z < 24; z++)
        {
            for (int y = 0; y < 8; y++)
            {
                for (int x = 16; x < 32; x++)
                {
                    svo.setVoxel({ x, y, z }, true);
                }
            }
        }

        for (int z = 8; z < 24; z++)
        {
            for (int y = 0; y < 12; y++)
            {
                for (int x = 32; x < 48; x++)
                {
                    svo.setVoxel({ x, y, z }, true);
                }
            }
        }

        MeshChunk chunkA;
        chunkA.minBound      = glm::ivec3(0, 0, 0);
        chunkA.size          = 32;
        chunkA.maxResolution = 32;

        MeshChunk chunkB;
        chunkB.minBound      = glm::ivec3(32, 0, 0);
        chunkB.size          = 32;
        chunkB.maxResolution = 16;

        std::vector<MeshChunk> chunks = { chunkA, chunkB };
        SVOCPUMesher::meshChunks(svo, chunks);

        const MeshChunk* seam = findBorderChunk(chunks, glm::ivec3(0, 0, 0), 0);
        REQUIRE(seam != nullptr);
        CHECK(computeDirQuadArea(*seam, 1) == 0);
        CHECK(computeDirQuadArea(*seam, 0) == 4 * 16);

        for (const auto& q : seam->quads)
        {
            CHECK(q.getDirection() == 0);
            CHECK(q.getX() == 32);
            CHECK(q.getY() >= 8);
            CHECK(q.getY() + q.getScale() <= 12);
        }
    }

    TEST_CASE("SVOMesher/LODDiffTwoLevelsDifferenceSubdivision", "[SVOMesher][Watertight][Boundary]")
    {
        SVO<bool> svo(64, false);

        for (int z = 8; z < 24; z++)
        {
            for (int y = 8; y < 24; y++)
            {
                for (int x = 32; x < 48; x++)
                {
                    svo.setVoxel({ x, y, z }, true);
                }
            }
        }

        MeshChunk chunkA;
        chunkA.minBound      = glm::ivec3(0, 0, 0);
        chunkA.size          = 32;
        chunkA.maxResolution = 32;

        MeshChunk chunkB;
        chunkB.minBound      = glm::ivec3(32, 0, 0);
        chunkB.size          = 32;
        chunkB.maxResolution = 8;

        std::vector<MeshChunk> chunks = { chunkA, chunkB };
        SVOCPUMesher::meshChunks(svo, chunks);

        CHECK(chunks[0].getCellSize() == 1);
        CHECK(chunks[1].getCellSize() == 4);

        const MeshChunk* seam = findBorderChunk(chunks, glm::ivec3(0, 0, 0), 0);
        REQUIRE(seam != nullptr);
        CHECK(computeDirQuadArea(*seam, 0) == 16 * 16);
    }

    TEST_CASE("SVOMesher/LODDiffPartialCoarseSolidInterface", "[SVOMesher][Watertight][Boundary]")
    {
        SVO<bool> svo(64, false);

        for (int z = 0; z < 2; z++)
        {
            for (int y = 0; y < 2; y++)
            {
                for (int x = 32; x < 34; x++)
                {
                    svo.setVoxel({ x, y, z }, true);
                }
            }
        }

        svo.setVoxel({ 31, 0, 0 }, true);
        svo.setVoxel({ 31, 1, 0 }, true);

        MeshChunk chunkA;
        chunkA.minBound      = glm::ivec3(0, 0, 0);
        chunkA.size          = 32;
        chunkA.maxResolution = 32;

        MeshChunk chunkB;
        chunkB.minBound      = glm::ivec3(32, 0, 0);
        chunkB.size          = 32;
        chunkB.maxResolution = 16;

        std::vector<MeshChunk> chunks = { chunkA, chunkB };
        SVOCPUMesher::meshChunks(svo, chunks);

        const MeshChunk* seam = findBorderChunk(chunks, glm::ivec3(0, 0, 0), 0);
        REQUIRE(seam != nullptr);
        CHECK(computeDirQuadArea(*seam, 1) == 0);
        CHECK(computeDirQuadArea(*seam, 0) == 2);

        for (const auto& q : seam->quads)
        {
            CHECK(q.getDirection() == 0);
            CHECK(q.getX() == 32);
            CHECK(q.getScale() == 1);
            CHECK(q.getZ() == 1);
        }
    }

    TEST_CASE("SVOMesher/LODDiffAllAxesWatertight", "[SVOMesher][Watertight][Boundary]")
    {
        SVO<bool> svo(64, false);

        for (int z = 16; z < 48; z++)
        {
            for (int y = 16; y < 48; y++)
            {
                for (int x = 16; x < 48; x++)
                {
                    svo.setVoxel({ x, y, z }, true);
                }
            }
        }

        std::vector<MeshChunk> chunks;
        for (int z = 0; z < 64; z += 32)
        {
            for (int y = 0; y < 64; y += 32)
            {
                for (int x = 0; x < 64; x += 32)
                {
                    MeshChunk c;
                    c.minBound      = glm::ivec3(x, y, z);
                    c.size          = 32;
                    c.maxResolution = (x == 0 && y == 0 && z == 0) ? 32 : 16;
                    chunks.push_back(c);
                }
            }
        }

        SVOCPUMesher::meshChunks(svo, chunks);

        auto mesh = SVOCPUMesher::extractTriangulation(chunks);
        REQUIRE(mesh != nullptr);
        CHECK(!mesh->vertices.empty());
        CHECK(!mesh->indices.empty());
        CHECK(mesh->indices.size() % 6 == 0);

        auto [minB, sz] = mesh->getAABB();
        CHECK(minB.x == Catch::Approx(16.0));
        CHECK(minB.y == Catch::Approx(16.0));
        CHECK(minB.z == Catch::Approx(16.0));
        CHECK(minB.x + sz.x == Catch::Approx(48.0));
        CHECK(minB.y + sz.y == Catch::Approx(48.0));
        CHECK(minB.z + sz.z == Catch::Approx(48.0));
    }

    TEST_CASE("SVOMesher/MultiStepLODLandscapeFullVisualWatertightness", "[SVOMesher][Watertight][Boundary]")
    {
        LandscapeConfig config;
        config.seed            = 1337;
        config.octaves         = 2;
        config.baseHeightRatio = 0.5;

        auto svo = SVOLandscapeGenerator::generateSVO<bool>(64, config);
        REQUIRE(svo != nullptr);

        const int lodAssignments[8] = { 0, 3, 1, 4, 2, 0, 3, 1 };

        std::vector<MeshChunk> chunks;
        int                    chunkIdx = 0;
        for (int z = 0; z < 64; z += 32)
        {
            for (int y = 0; y < 64; y += 32)
            {
                for (int x = 0; x < 64; x += 32)
                {
                    MeshChunk c;
                    c.minBound      = glm::ivec3(x, y, z);
                    c.size          = 32;
                    c.maxResolution = 32;
                    c.lodLevel      = lodAssignments[chunkIdx];
                    chunkIdx++;
                    chunks.push_back(c);
                }
            }
        }

        std::vector<uint8_t> renderedSolid(64 * 64 * 64, 0);
        auto                 voxelIdx = [](int x, int y, int z) -> size_t { return static_cast<size_t>(z * 64 * 64 + y * 64 + x); };

        for (size_t i = 0; i < 8; i++)
        {
            uint32_t          cellSize = chunks[i].getCellSize();
            SVODualNode<bool> root     = SVODualTraversal::findSubtreeNode(*svo, chunks[i].minBound, chunks[i].size);
            SVODualTraversal::traverseSubtree(
              *svo,
              root,
              [&](const SVODualNode<bool>& node) { return node.size <= cellSize; },
              [&](const SVODualNode<bool>& leaf)
              {
                  if (!leaf.value)
                  {
                      return;
                  }
                  for (uint32_t dz = 0; dz < leaf.size; dz++)
                  {
                      for (uint32_t dy = 0; dy < leaf.size; dy++)
                      {
                          for (uint32_t dx = 0; dx < leaf.size; dx++)
                          {
                              renderedSolid[voxelIdx(leaf.offset.x + dx, leaf.offset.y + dy, leaf.offset.z + dz)] = 1;
                          }
                      }
                  }
              },
              [](const SVODualNode<bool>&, const SVODualNode<bool>&, SVOAxis) {});
        }

        SVOCPUMesher::meshChunks(*svo, chunks);

        std::vector<uint32_t> faceCoverage(6 * 65 * 64 * 64, 0);
        auto                  faceIdx = [](uint32_t dir, int plane, int u, int v) -> size_t { return static_cast<size_t>(((dir * 65 + plane) * 64 + u) * 64 + v); };

        for (const auto& chunk : chunks)
        {
            for (const auto& q : chunk.quads)
            {
                glm::ivec3 pos = chunk.minBound + glm::ivec3(q.getLocalPos());
                int        s   = static_cast<int>(q.getScale());
                uint32_t   dir = q.getDirection();

                if (dir == 0 || dir == 1)
                {
                    int plane = (dir == 0) ? pos.x : (pos.x + s);
                    for (int dz = 0; dz < s; dz++)
                    {
                        for (int dy = 0; dy < s; dy++)
                        {
                            faceCoverage[faceIdx(dir, plane, pos.y + dy, pos.z + dz)]++;
                        }
                    }
                }
                else if (dir == 2 || dir == 3)
                {
                    int plane = (dir == 2) ? pos.y : (pos.y + s);
                    for (int dz = 0; dz < s; dz++)
                    {
                        for (int dx = 0; dx < s; dx++)
                        {
                            faceCoverage[faceIdx(dir, plane, pos.x + dx, pos.z + dz)]++;
                        }
                    }
                }
                else if (dir == 4 || dir == 5)
                {
                    int plane = (dir == 4) ? pos.z : (pos.z + s);
                    for (int dy = 0; dy < s; dy++)
                    {
                        for (int dx = 0; dx < s; dx++)
                        {
                            faceCoverage[faceIdx(dir, plane, pos.x + dx, pos.y + dy)]++;
                        }
                    }
                }
            }
        }

        auto isSolidAt = [&](int x, int y, int z) -> bool
        {
            if (x < 0 || x >= 64 || y < 0 || y >= 64 || z < 0 || z >= 64)
            {
                return false;
            }
            return renderedSolid[voxelIdx(x, y, z)] != 0;
        };

        size_t missingFaces     = 0;
        size_t overlappingFaces = 0;
        size_t internalFaces    = 0;

        for (int plane = 0; plane <= 64; plane++)
        {
            for (int u = 0; u < 64; u++)
            {
                for (int v = 0; v < 64; v++)
                {
                    bool     solidNegX = isSolidAt(plane - 1, u, v);
                    bool     solidPosX = isSolidAt(plane, u, v);
                    uint32_t covNegX   = faceCoverage[faceIdx(0, plane, u, v)];
                    uint32_t covPosX   = faceCoverage[faceIdx(1, plane, u, v)];
                    if (!solidNegX && solidPosX && covNegX == 0)
                        missingFaces++;
                    if (solidNegX && !solidPosX && covPosX == 0)
                        missingFaces++;
                    if (covNegX > 1)
                        overlappingFaces++;
                    if (covPosX > 1)
                        overlappingFaces++;
                    if ((solidNegX || !solidPosX) && covNegX > 0)
                        internalFaces++;
                    if ((!solidNegX || solidPosX) && covPosX > 0)
                        internalFaces++;

                    bool     solidNegY = isSolidAt(u, plane - 1, v);
                    bool     solidPosY = isSolidAt(u, plane, v);
                    uint32_t covNegY   = faceCoverage[faceIdx(2, plane, u, v)];
                    uint32_t covPosY   = faceCoverage[faceIdx(3, plane, u, v)];
                    if (!solidNegY && solidPosY && covNegY == 0)
                        missingFaces++;
                    if (solidNegY && !solidPosY && covPosY == 0)
                        missingFaces++;
                    if (covNegY > 1)
                        overlappingFaces++;
                    if (covPosY > 1)
                        overlappingFaces++;
                    if ((solidNegY || !solidPosY) && covNegY > 0)
                        internalFaces++;
                    if ((!solidNegY || solidPosY) && covPosY > 0)
                        internalFaces++;

                    bool     solidNegZ = isSolidAt(u, v, plane - 1);
                    bool     solidPosZ = isSolidAt(u, v, plane);
                    uint32_t covNegZ   = faceCoverage[faceIdx(4, plane, u, v)];
                    uint32_t covPosZ   = faceCoverage[faceIdx(5, plane, u, v)];
                    if (!solidNegZ && solidPosZ && covNegZ == 0)
                        missingFaces++;
                    if (solidNegZ && !solidPosZ && covPosZ == 0)
                        missingFaces++;
                    if (covNegZ > 1)
                        overlappingFaces++;
                    if (covPosZ > 1)
                        overlappingFaces++;
                    if ((solidNegZ || !solidPosZ) && covNegZ > 0)
                        internalFaces++;
                    if ((!solidNegZ || solidPosZ) && covPosZ > 0)
                        internalFaces++;
                }
            }
        }

        CHECK(missingFaces == 0);
        CHECK(overlappingFaces == 0);
        CHECK(internalFaces == 0);
    }
}
#endif

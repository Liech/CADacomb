#include "SVOBlockyTriangulation.h"

namespace Library::Volume
{
    SVOBlockyTriangulation::SVOBlockyTriangulation() {}

    SVOBlockyTriangulation::~SVOBlockyTriangulation() {}
}

#ifdef ISTESTPROJECT
#include "CADacomb/Volume/MergeStrategy/MostPrevalentExceptMergeStrategy.h"
#include "CADacomb/Volume/MergeStrategy/MostPrevalentMergeStrategy.h"
#include <catch2/catch_test_macros.hpp>

namespace Library::Volume
{
    TEST_CASE("SVOBlocky/EmptyVolume", "[SVO][Blocky]")
    {
        SVO<bool>        svo(8, false);
        SVOBlockyOptions options;
        auto             mesh = SVOBlockyTriangulation::triangulate(svo, options);

        REQUIRE(mesh != nullptr);
        CHECK(mesh->vertices.empty());
        CHECK(mesh->indices.empty());
    }

    TEST_CASE("SVOBlocky/SingleVoxelCube", "[SVO][Blocky]")
    {
        SVO<bool> svo(8, false);
        svo.setVoxel({ 2, 2, 2 }, true);

        SVOBlockyOptions options;
        options.targetDepth = 3;
        options.origin      = glm::dvec3(0.0);
        options.size        = glm::dvec3(8.0);
        auto mesh           = SVOBlockyTriangulation::triangulate(svo, options);

        REQUIRE(mesh != nullptr);
        CHECK(mesh->vertices.size() == 24);
        CHECK(mesh->indices.size() == 36);

        auto [minB, sz] = mesh->getAABB();
        CHECK(std::abs(minB.x - 2.0) < 1e-4);
        CHECK(std::abs(minB.y - 2.0) < 1e-4);
        CHECK(std::abs(minB.z - 2.0) < 1e-4);
        CHECK(std::abs(sz.x - 1.0) < 1e-4);
        CHECK(std::abs(sz.y - 1.0) < 1e-4);
        CHECK(std::abs(sz.z - 1.0) < 1e-4);
    }

    TEST_CASE("SVOBlocky/InternalFacesCulled", "[SVO][Blocky]")
    {
        SVO<bool> svo(8, false);
        svo.setVoxel({ 2, 2, 2 }, true);
        svo.setVoxel({ 3, 2, 2 }, true);

        SVOBlockyOptions options;
        options.targetDepth = 3;
        options.origin      = glm::dvec3(0.0);
        options.size        = glm::dvec3(8.0);
        auto mesh           = SVOBlockyTriangulation::triangulate(svo, options);

        REQUIRE(mesh != nullptr);
        CHECK(mesh->vertices.size() == 40);
        CHECK(mesh->indices.size() == 60);

        auto [minB, sz] = mesh->getAABB();
        CHECK(std::abs(minB.x - 2.0) < 1e-4);
        CHECK(std::abs(sz.x - 2.0) < 1e-4);
        CHECK(std::abs(sz.y - 1.0) < 1e-4);
        CHECK(std::abs(sz.z - 1.0) < 1e-4);
    }

    TEST_CASE("SVOBlocky/LODLevelsPreserveBounds", "[SVO][Blocky]")
    {
        SVO<bool> svo(8, false);
        for (int y = 0; y < 4; ++y)
            for (int z = y; z < 8 - y; ++z)
                for (int x = y; x < 8 - y; ++x)
                    svo.setVoxel({ x, y, z }, true);

        SVOBlockyOptions optLOD3;
        optLOD3.targetDepth = 3;
        optLOD3.origin      = glm::dvec3(0.0);
        optLOD3.size        = glm::dvec3(8.0);
        auto mesh3          = SVOBlockyTriangulation::triangulate(svo, optLOD3);

        SVOBlockyOptions optLOD2;
        optLOD2.targetDepth = 2;
        optLOD2.origin      = glm::dvec3(0.0);
        optLOD2.size        = glm::dvec3(8.0);
        auto mesh2          = SVOBlockyTriangulation::triangulate(svo, optLOD2);

        REQUIRE(mesh3 != nullptr);
        REQUIRE(mesh2 != nullptr);

        CHECK(mesh3->indices.size() > mesh2->indices.size());
    }

    TEST_CASE("SVOBlocky/AdaptiveDistanceLOD", "[SVO][Blocky]")
    {
        SVO<bool> svo(16, false);
        for (int y = 0; y < 8; ++y)
            for (int z = y; z < 16 - y; ++z)
                for (int x = y; x < 16 - y; ++x)
                    svo.setVoxel({ x, y, z }, true);

        SVOBlockyOptions opt;
        opt.adaptive        = true;
        opt.focalPoint      = glm::dvec3(0.0);
        opt.lodNearDistance = 1.0;
        opt.lodFarDistance  = 16.0;
        opt.minDepth        = 1;
        opt.maxDepth        = 4;
        opt.origin          = glm::dvec3(0.0);
        opt.size            = glm::dvec3(16.0);

        auto mesh = SVOBlockyTriangulation::triangulate(svo, opt);
        REQUIRE(mesh != nullptr);
        CHECK(!mesh->vertices.empty());
        CHECK(!mesh->indices.empty());
        CHECK(mesh->indices.size() % 6 == 0);

        SVOBlockyOptions optUniform;
        optUniform.targetDepth = 4;
        optUniform.origin      = glm::dvec3(0.0);
        optUniform.size        = glm::dvec3(16.0);
        auto meshUniform       = SVOBlockyTriangulation::triangulate(svo, optUniform);
        REQUIRE(meshUniform != nullptr);

        CHECK(mesh->indices.size() < meshUniform->indices.size());

        auto [minB, sz] = mesh->getAABB();
        CHECK(std::abs(minB.x - 0.0) < 1e-4);
        CHECK(std::abs(minB.y - 0.0) < 1e-4);
        CHECK(std::abs(minB.z - 0.0) < 1e-4);
        CHECK(std::abs(sz.x - 16.0) < 1e-4);
        CHECK(std::abs(sz.y - 8.0) < 1e-4);
        CHECK(std::abs(sz.z - 16.0) < 1e-4);
    }

    TEST_CASE("SVODualTraversal/SingleVoxelFaces", "[SVO][DualTraversal]")
    {
        SVO<bool> svo(8, false);
        svo.setVoxel({ 2, 2, 2 }, true);

        int exposedFaceCount  = 0;
        int internalFaceCount = 0;

        auto isLeaf = [](const SVODualNode<bool>& node) { return node.isLeaf; };
        auto onNode = [](const SVODualNode<bool>&) {};
        auto onFace = [&](const SVODualNode<bool>& nodeA, const SVODualNode<bool>& nodeB, SVOAxis)
        {
            bool solidA = !nodeA.isVirtual && nodeA.value;
            bool solidB = !nodeB.isVirtual && nodeB.value;
            if (solidA != solidB)
            {
                exposedFaceCount++;
            }
            else if (solidA && solidB)
            {
                internalFaceCount++;
            }
        };

        SVODualTraversal::traverse(svo, isLeaf, onNode, onFace);

        CHECK(exposedFaceCount == 6);
        CHECK(internalFaceCount == 0);
    }

    TEST_CASE("SVODualTraversal/TwoAdjacentVoxelsCullInternal", "[SVO][DualTraversal]")
    {
        SVO<bool> svo(8, false);
        svo.setVoxel({ 2, 2, 2 }, true);
        svo.setVoxel({ 3, 2, 2 }, true);

        int exposedFaceCount       = 0;
        int internalSolidFaceCount = 0;

        auto isLeaf = [](const SVODualNode<bool>& node) { return node.isLeaf; };
        auto onNode = [](const SVODualNode<bool>&) {};
        auto onFace = [&](const SVODualNode<bool>& nodeA, const SVODualNode<bool>& nodeB, SVOAxis)
        {
            bool solidA = !nodeA.isVirtual && nodeA.value;
            bool solidB = !nodeB.isVirtual && nodeB.value;
            if (solidA != solidB)
            {
                exposedFaceCount++;
            }
            else if (solidA && solidB)
            {
                internalSolidFaceCount++;
            }
        };

        SVODualTraversal::traverse(svo, isLeaf, onNode, onFace);

        CHECK(internalSolidFaceCount == 1);
        CHECK(exposedFaceCount == 10);
    }

    TEST_CASE("SVOBlocky/MostPrevalentExceptEmptiness", "[SVO][Blocky][Strategy]")
    {
        SVO<bool> svo(4, false, MostPrevalentExceptMergeStrategy<bool, false>::instance());
        svo.setVoxel({ 0, 0, 0 }, true);

        SVOBlockyOptions optLOD1;
        optLOD1.targetDepth = 1;
        optLOD1.origin      = glm::dvec3(0.0);
        optLOD1.size        = glm::dvec3(4.0);
        auto mesh           = SVOBlockyTriangulation::triangulate(svo, optLOD1);

        REQUIRE(mesh != nullptr);
        CHECK(!mesh->vertices.empty());
        CHECK(mesh->indices.size() == 36);
    }
}
#endif

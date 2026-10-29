#include "CADacombRender/SVOLODMesh.h"

#ifdef ISTESTPROJECT
#include "CADacomb/Volume/LODStrategy/FixedLODStrategy.h"
#include "Vulcant/VulcantV/VulcantVDevice.h"
#include <catch2/catch_test_macros.hpp>

namespace CADacomb::Render
{
    TEST_CASE("SVOLODMesh/LifecycleAndRemeshing", "[SVOLODMesh]")
    {
        Vulcant::VulcantV::VulcantVDevice device({}, false);

        CADacomb::SVO<bool> svo(64, false);
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

        CADacomb::AABB3  volumeBounds(glm::dvec3(0.0), glm::dvec3(64.0));
        SVOLODMesh<bool> lodMesh(device, svo, volumeBounds, 20000, 32);

        CHECK(lodMesh.getChunks().size() == 8);
        CHECK(lodMesh.getVertexCount() == 0);

        Vulcant::Toolkit::SceneData sceneData{};
        sceneData.resolution             = glm::vec2(1920.0f, 1080.0f);
        sceneData.projectionMatrix       = glm::mat4(1.0f);
        sceneData.projectionMatrix[1][1] = 1.7320508f;
        sceneData.cameraPos              = glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);

        CADacomb::FixedLODStrategy refineStrategy(CADacomb::LODDecision::Refine);
        while (lodMesh.getVertexCount() == 0 && lodMesh.getChunks()[0].getLOD() > 0)
        {
            lodMesh.update(sceneData, refineStrategy);
        }

        CHECK(lodMesh.getVertexCount() > 0);
        size_t initialVertexCount = lodMesh.getVertexCount();

        for (int z = 4; z < 12; z++)
        {
            for (int y = 4; y < 12; y++)
            {
                for (int x = 4; x < 12; x++)
                {
                    svo.setVoxel({ x, y, z }, false);
                }
            }
        }

        lodMesh.markDirty(CADacomb::AABB3(glm::dvec3(4.0), glm::dvec3(12.0)));

        CADacomb::FixedLODStrategy keepStrategy(CADacomb::LODDecision::Keep);
        lodMesh.update(sceneData, keepStrategy);

        CHECK(lodMesh.getVertexCount() == 0);
        CHECK(lodMesh.getVertexCount() != initialVertexCount);
    }

    class SelectiveChunkLODStrategy : public CADacomb::LODStrategy
    {
      public:
        SelectiveChunkLODStrategy() {}
        virtual ~SelectiveChunkLODStrategy() {}

        virtual void startFrame(const CADacomb::SceneData& sceneData, const CADacomb::AABB3& volumeBounds, size_t maxAllowedVertices, size_t svoDimension) override
        {
            (void)sceneData;
            (void)volumeBounds;
            (void)maxAllowedVertices;
            (void)svoDimension;
        }

        virtual CADacomb::LODDecision evaluate(const CADacomb::AABB3& chunkBounds, double currentVoxelLength, int currentLOD, size_t currentTotalVertices) override
        {
            (void)currentVoxelLength;
            (void)currentLOD;
            (void)currentTotalVertices;
            if (chunkBounds.getMin().x >= 32.0 && chunkBounds.getMin().y == 0.0 && chunkBounds.getMin().z == 0.0)
            {
                return coarsenTarget ? CADacomb::LODDecision::Coarsen : CADacomb::LODDecision::Keep;
            }
            return CADacomb::LODDecision::Keep;
        }

        virtual void endFrame() override {}

        bool coarsenTarget = true;
    };

    TEST_CASE("SVOLODMesh/IndependentBorderChunkRemeshingOnLODChange", "[SVOLODMesh][Watertight]")
    {
        Vulcant::VulcantV::VulcantVDevice device({}, false);

        CADacomb::SVO<bool> svo(64, false);
        for (int z = 8; z < 24; z++)
        {
            for (int y = 8; y < 24; y++)
            {
                for (int x = 16; x < 48; x++)
                {
                    svo.setVoxel({ x, y, z }, true);
                }
            }
        }

        CADacomb::AABB3  volumeBounds(glm::dvec3(0.0), glm::dvec3(64.0));
        SVOLODMesh<bool> lodMesh(device, svo, volumeBounds, 100000, 32, 0);

        CHECK(lodMesh.getChunks().size() == 8);
        CHECK(lodMesh.getBorderChunks().size() == 36);

        Vulcant::Toolkit::SceneData sceneData{};
        CADacomb::FixedLODStrategy  keepStrategy(CADacomb::LODDecision::Keep);
        lodMesh.update(sceneData, keepStrategy);

        size_t chunk0PackageBefore = lodMesh.getChunks()[0].getPackageId();
        CHECK(lodMesh.getChunks()[0].hasPackage());
        CHECK(lodMesh.getChunks()[1].hasPackage());

        SelectiveChunkLODStrategy selectiveStrategy;
        lodMesh.update(sceneData, selectiveStrategy);
        lodMesh.update(sceneData, selectiveStrategy);
        lodMesh.update(sceneData, selectiveStrategy);

        CHECK(lodMesh.getChunks()[0].getLOD() == 0);
        CHECK(lodMesh.getChunks()[1].getLOD() == 3);
        CHECK(lodMesh.getChunks()[0].getPackageId() == chunk0PackageBefore);
        CHECK(lodMesh.getVertexCount() > 0);
    }

    TEST_CASE("SVOLODMesh/StartsAtCoarsestLODByDefault", "[SVOLODMesh]")
    {
        Vulcant::VulcantV::VulcantVDevice device({}, false);
        CADacomb::SVO<bool>               svo(64, false);
        CADacomb::AABB3                   volumeBounds(glm::dvec3(0.0), glm::dvec3(64.0));
        SVOLODMesh<bool>                  lodMesh(device, svo, volumeBounds, 20000, 32);

        for (const auto& chunk : lodMesh.getChunks())
        {
            CHECK(chunk.getLOD() == 5);
        }
        for (const auto& border : lodMesh.getBorderChunks())
        {
            CHECK(border.getLOD() == 5);
        }
    }
}
#endif

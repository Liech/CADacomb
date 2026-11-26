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
        lodMesh.setAsyncEnabled(false);

        CHECK(lodMesh.getChunks().size() == 1);
        CHECK(lodMesh.getVertexCount() == 0);

        Vulcant::Toolkit::SceneData sceneData{};
        sceneData.resolution             = glm::vec2(1920.0f, 1080.0f);
        sceneData.projectionMatrix       = glm::mat4(1.0f);
        sceneData.projectionMatrix[1][1] = 1.7320508f;
        sceneData.cameraPos              = glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);

        CADacomb::FixedLODStrategy refineStrategy(CADacomb::LODDecision::Refine);
        for (int step = 0; step < 8 && lodMesh.getVertexCount() == 0; step++)
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

    TEST_CASE("SVOLODMesh/StartsAtCoarsestLODByDefault", "[SVOLODMesh]")
    {
        Vulcant::VulcantV::VulcantVDevice device({}, false);
        CADacomb::SVO<bool>               svo(64, false);
        CADacomb::AABB3                   volumeBounds(glm::dvec3(0.0), glm::dvec3(64.0));
        SVOLODMesh<bool>                  lodMesh(device, svo, volumeBounds, 20000, 32);

        REQUIRE(lodMesh.getChunks().size() == 1);
        CHECK(lodMesh.getChunks()[0].getLOD() == 6);
    }
}
#endif

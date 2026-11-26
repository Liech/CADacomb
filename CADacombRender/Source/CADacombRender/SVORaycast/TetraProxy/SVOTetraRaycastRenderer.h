#pragma once

#include "CADacomb/Util/AABB3.h"
#include "CADacomb/Volume/SVO/SVO.h"
#include "CADacombRender/SVORaycast/Pipeline/SVORaycastParams.h"
#include "CADacombRender/SVORaycast/TetraProxy/GPUTetraVertex.h"
#include "CADacombRender/SVORaycast/TetraProxy/GPUTetrahedron.h"
#include "CADacombRender/SVORaycast/TetraProxy/TetraSurfaceVertex.h"
#include "Vulcant/Toolkit/SceneData.h"

#include <cstdint>
#include <glm/glm.hpp>
#include <memory>
#include <vector>

namespace Vulcant
{
    class VulcantBuffer;
    class VulcantDevice;
    class VulcantGraphicCommand;
    class VulcantGraphicPipeline;
    class VulcantImage;
    class VulcantSet;
    class VulcantShader;
}

namespace CADacomb::Render
{
    // Standalone deformable GPU SVO raycast renderer using a conforming tetrahedron cage proxy.
    class SVOTetraRaycastRenderer
    {
      public:
        explicit SVOTetraRaycastRenderer(Vulcant::VulcantDevice& deviceInput);
        virtual ~SVOTetraRaycastRenderer();

        void uploadSVO(const CADacomb::SVO<bool>& svo, const CADacomb::AABB3& volumeBounds, uint32_t cageGridResolution = 4u);
        void deformSine(const CADacomb::AABB3& volumeBounds, float timeSeconds, float amplitudeFactor = 0.08f);
        void uploadDeformedVertices(const std::vector<GPUTetraVertex>& verticesInput);

        void render(Vulcant::VulcantImage&             colorTarget,
                    Vulcant::VulcantImage&             normalTarget,
                    Vulcant::VulcantImage&             depthTarget,
                    const Vulcant::Toolkit::SceneData& worldSceneData,
                    const CADacomb::AABB3&             volumeBounds,
                    const glm::dmat4&                  transformation,
                    const glm::vec3&                   baseColor,
                    float                              pixelLodScale = 1.0f,
                    uint32_t                           maxDepth      = 0);

        size_t   getTetrahedronCount() const;
        uint32_t getUploadedDimension() const;

      private:
        void    initializeResources();
        void    ensurePipeline(Vulcant::VulcantImage& colorTarget, Vulcant::VulcantImage& normalTarget, Vulcant::VulcantImage& depthTarget);
        void    invalidatePipeline();
        int32_t findCameraTetrahedron(const glm::vec3& localCameraPos) const;

        Vulcant::VulcantDevice&                          device;
        std::unique_ptr<Vulcant::VulcantShader>          vertShader;
        std::unique_ptr<Vulcant::VulcantShader>          fragShader;
        std::unique_ptr<Vulcant::VulcantGraphicPipeline> pipeline;
        std::unique_ptr<Vulcant::VulcantSet>             descriptorSet;
        std::unique_ptr<Vulcant::VulcantBuffer>          surfaceVertexBuffer;
        std::unique_ptr<Vulcant::VulcantBuffer>          indirectBuffer;
        std::unique_ptr<Vulcant::VulcantBuffer>          sceneDataUbo;
        std::unique_ptr<Vulcant::VulcantBuffer>          raycastParamsUbo;
        std::unique_ptr<Vulcant::VulcantBuffer>          svoNodeBuffer;
        std::unique_ptr<Vulcant::VulcantBuffer>          tetraVertexBuffer;
        std::unique_ptr<Vulcant::VulcantBuffer>          tetraBuffer;
        std::unique_ptr<Vulcant::VulcantGraphicCommand>  drawCmd;
        std::vector<GPUTetraVertex>                      cageVertices;
        std::vector<GPUTetrahedron>                      cageTetrahedra;
        std::vector<TetraSurfaceVertex>                  surfaceVertices;
        SVORaycastParams                                 raycastParams;
        const Vulcant::VulcantImage*                     lastColorTarget   = nullptr;
        glm::ivec2                                       lastResolution{ 0, 0 };
        uint32_t                                         uploadedDimension = 0;
    };
}

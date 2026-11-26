#pragma once

#include "CADacomb/Util/AABB3.h"
#include "CADacomb/Volume/SVO/SVO.h"
#include "CADacombRender/SVORaycast/Pipeline/SVORaycastParams.h"
#include "Vulcant/Toolkit/SceneData.h"

#include <cstdint>
#include <glm/glm.hpp>
#include <memory>

namespace Vulcant
{
    class VulcantBuffer;
    class VulcantComputeCommand;
    class VulcantDevice;
    class VulcantGraphicCommand;
    class VulcantGraphicPipeline;
    class VulcantImage;
    class VulcantSet;
    class VulcantShader;
}

namespace CADacomb::Render
{
    // Standalone GPU SVO raycast renderer using modular Slang shaders, coarse beam prepass, and proxy volume rasterization.
    class SVORaycastRenderer
    {
      public:
        explicit SVORaycastRenderer(Vulcant::VulcantDevice& deviceInput);
        virtual ~SVORaycastRenderer();

        void uploadSVO(const CADacomb::SVO<bool>& svo);

        void render(Vulcant::VulcantImage&             colorTarget,
                    Vulcant::VulcantImage&             normalTarget,
                    Vulcant::VulcantImage&             depthTarget,
                    const Vulcant::Toolkit::SceneData& worldSceneData,
                    const CADacomb::AABB3&             volumeBounds,
                    const glm::dmat4&                  transformation,
                    const glm::vec3&                   baseColor,
                    float                              pixelLodScale = 1.0f,
                    uint32_t                           maxDepth      = 0);

        size_t   getUploadedNodeCount() const;
        uint32_t getUploadedDimension() const;

      private:
        void initializeResources();
        void ensurePipeline(Vulcant::VulcantImage& colorTarget, Vulcant::VulcantImage& normalTarget, Vulcant::VulcantImage& depthTarget);
        void invalidatePipeline();

        Vulcant::VulcantDevice&                          device;
        std::unique_ptr<Vulcant::VulcantShader>          vertShader;
        std::unique_ptr<Vulcant::VulcantShader>          fragShader;
        std::unique_ptr<Vulcant::VulcantShader>          beamShader;
        std::unique_ptr<Vulcant::VulcantGraphicPipeline> pipeline;
        std::unique_ptr<Vulcant::VulcantSet>             descriptorSet;
        std::unique_ptr<Vulcant::VulcantSet>             beamDescriptorSet;
        std::unique_ptr<Vulcant::VulcantBuffer>          proxyVertexBuffer;
        std::unique_ptr<Vulcant::VulcantBuffer>          indirectBuffer;
        std::unique_ptr<Vulcant::VulcantBuffer>          sceneDataUbo;
        std::unique_ptr<Vulcant::VulcantBuffer>          raycastParamsUbo;
        std::unique_ptr<Vulcant::VulcantBuffer>          svoNodeBuffer;
        std::unique_ptr<Vulcant::VulcantBuffer>          beamBuffer;
        std::unique_ptr<Vulcant::VulcantComputeCommand>  beamCmd;
        std::unique_ptr<Vulcant::VulcantGraphicCommand>  drawCmd;
        SVORaycastParams                                 raycastParams;
        const Vulcant::VulcantImage*                     lastColorTarget = nullptr;
        glm::ivec2                                       lastResolution{ 0, 0 };
        uint32_t                                         beamWidth         = 0;
        uint32_t                                         beamHeight        = 0;
        uint32_t                                         beamTileSize      = 8;
        size_t                                           uploadedNodeCount = 0;
        uint32_t                                         uploadedDimension = 0;
    };
}

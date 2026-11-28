#pragma once

#include "CADacomb/SDF/Sdf.h"
#include "CADacomb/Util/AABB3.h"
#include "CADacombRender/SDFRaycast/SdfRaycastParams.h"
#include "Vulcant/Toolkit/SceneData.h"

#include <glm/glm.hpp>
#include <memory>
#include <string>

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
    // GPU sphere-tracing renderer for CADacomb::Sdf using runtime-compiled Slang shaders.
    class SdfRaycastRenderer
    {
      public:
        explicit SdfRaycastRenderer(Vulcant::VulcantDevice& deviceInput);
        virtual ~SdfRaycastRenderer();

        bool setSdf(const CADacomb::Sdf& sdf, std::string& outError);

        void render(Vulcant::VulcantImage&             colorTarget,
                    Vulcant::VulcantImage&             normalTarget,
                    Vulcant::VulcantImage&             depthTarget,
                    const Vulcant::Toolkit::SceneData& worldSceneData,
                    const CADacomb::AABB3&             volumeBounds,
                    const glm::dmat4&                  transformation,
                    const glm::vec3&                   baseColor,
                    uint32_t                           maxSteps        = 128u,
                    float                              hitEpsilon      = 0.0005f,
                    float                              stepScale       = 1.0f,
                    float                              normalEpsilon   = 0.001f,
                    float                              ambientStrength = 0.25f);

        bool isReady() const;

      private:
        void               initializeResources();
        void               ensurePipeline(Vulcant::VulcantImage& colorTarget, Vulcant::VulcantImage& normalTarget, Vulcant::VulcantImage& depthTarget);
        void               invalidatePipeline();
        static std::string buildVertexShaderSource();
        static std::string buildFragmentShaderSource(const CADacomb::Sdf& sdf);

        Vulcant::VulcantDevice&                          device;
        std::unique_ptr<Vulcant::VulcantShader>          vertShader;
        std::unique_ptr<Vulcant::VulcantShader>          fragShader;
        std::unique_ptr<Vulcant::VulcantGraphicPipeline> pipeline;
        std::unique_ptr<Vulcant::VulcantSet>             descriptorSet;
        std::unique_ptr<Vulcant::VulcantBuffer>          proxyVertexBuffer;
        std::unique_ptr<Vulcant::VulcantBuffer>          indirectBuffer;
        std::unique_ptr<Vulcant::VulcantBuffer>          sceneDataUbo;
        std::unique_ptr<Vulcant::VulcantBuffer>          raycastParamsUbo;
        std::unique_ptr<Vulcant::VulcantGraphicCommand>  drawCmd;
        SdfRaycastParams                                 raycastParams;
        const Vulcant::VulcantImage*                     lastColorTarget = nullptr;
        glm::ivec2                                       lastResolution{ 0, 0 };
    };
}

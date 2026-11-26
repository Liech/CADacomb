#pragma once

#include <cstdint>
#include <glm/glm.hpp>

namespace CADacomb::Render
{
    // Uniform buffer payload matching Slang SVORaycastParams layout.
    struct SVORaycastParams
    {
      public:
        SVORaycastParams();

        void setTransformation(const glm::dmat4& transform);

        glm::mat4 modelMatrix{ 1.0f };
        glm::mat4 invModelMatrix{ 1.0f };
        glm::mat4 normalMatrix{ 1.0f };
        glm::vec4 aabbMin{ 0.0f };
        glm::vec4 aabbSize{ 1.0f };
        glm::vec4 baseColor{ 0.6f, 0.75f, 0.9f, 1.0f };
        uint32_t  svoDimension  = 1;
        uint32_t  maxSteps      = 512;
        uint32_t  maxDepth      = 0;
        float     pixelLodScale = 1.0f;
        uint32_t  beamWidth      = 0;
        uint32_t  beamHeight     = 0;
        uint32_t  beamTileSize   = 8;
        uint32_t  beamEnabled    = 1;
        int32_t   cameraTetIndex = -1;
        int32_t   padding0       = 0;
        int32_t   padding1       = 0;
        int32_t   padding2       = 0;
    };
}

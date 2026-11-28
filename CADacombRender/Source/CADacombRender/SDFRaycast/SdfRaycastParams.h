#pragma once

#include <cstdint>
#include <glm/glm.hpp>

namespace CADacomb::Render
{
    // Uniform buffer payload matching Slang SdfRaycastParams layout.
    struct SdfRaycastParams
    {
      public:
        SdfRaycastParams();

        void setTransformation(const glm::dmat4& transformation);

        glm::mat4 modelMatrix{ 1.0f };
        glm::mat4 invModelMatrix{ 1.0f };
        glm::mat4 normalMatrix{ 1.0f };
        glm::vec4 aabbMin{ -1.0f, -1.0f, -1.0f, 0.0f };
        glm::vec4 aabbSize{ 2.0f, 2.0f, 2.0f, 0.0f };
        glm::vec4 baseColor{ 0.72f, 0.78f, 0.86f, 1.0f };
        uint32_t  maxSteps        = 128u;
        float     hitEpsilon      = 0.0005f;
        float     stepScale       = 1.0f;
        float     normalEpsilon   = 0.001f;
        float     ambientStrength = 0.25f;
        float     pad0            = 0.0f;
        float     pad1            = 0.0f;
        float     pad2            = 0.0f;
    };
}

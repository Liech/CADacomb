#pragma once

#include <glm/glm.hpp>

namespace CADacomb
{
    class SceneData
    {
      public:
        SceneData();
        virtual ~SceneData();

        glm::dmat4 viewMatrix{ 1.0 };
        glm::dmat4 projectionMatrix{ 1.0 };
        glm::dmat4 invProjectionMatrix{ 1.0 };
        glm::dmat4 invViewMatrix{ 1.0 };
        glm::dvec4 cameraPos{ 0.0, 0.0, 0.0, 1.0 };
        glm::dvec2 resolution{ 0.0, 0.0 };
        double     nearPlane = 0.1;
        double     farPlane  = 1000.0;
    };
}

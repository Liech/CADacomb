#include "CADacombRender/SceneDataConverter.h"

namespace CADacomb::Render
{
    SceneDataConverter::SceneDataConverter() {}

    SceneDataConverter::~SceneDataConverter() {}

    CADacomb::SceneData SceneDataConverter::convert(const Vulcant::Toolkit::SceneData& sceneData)
    {
        CADacomb::SceneData result;
        result.viewMatrix          = glm::dmat4(sceneData.viewMatrix);
        result.projectionMatrix    = glm::dmat4(sceneData.projectionMatrix);
        result.invProjectionMatrix = glm::dmat4(sceneData.invProjectionMatrix);
        result.invViewMatrix       = glm::dmat4(sceneData.invViewMatrix);
        result.cameraPos           = glm::dvec4(sceneData.cameraPos);
        result.resolution          = glm::dvec2(sceneData.resolution);
        result.nearPlane           = static_cast<double>(sceneData.nearPlane);
        result.farPlane            = static_cast<double>(sceneData.farPlane);
        return result;
    }

    Vulcant::Toolkit::SceneData SceneDataConverter::convert(const CADacomb::SceneData& sceneData)
    {
        Vulcant::Toolkit::SceneData result;
        result.viewMatrix          = glm::mat4(sceneData.viewMatrix);
        result.projectionMatrix    = glm::mat4(sceneData.projectionMatrix);
        result.invProjectionMatrix = glm::mat4(sceneData.invProjectionMatrix);
        result.invViewMatrix       = glm::mat4(sceneData.invViewMatrix);
        result.cameraPos           = glm::vec4(sceneData.cameraPos);
        result.resolution          = glm::vec2(sceneData.resolution);
        result.nearPlane           = static_cast<float>(sceneData.nearPlane);
        result.farPlane            = static_cast<float>(sceneData.farPlane);
        return result;
    }
}

#include "CADacombRender/SDFRaycast/SdfRaycastParams.h"

#include <glm/gtc/matrix_inverse.hpp>

namespace CADacomb::Render
{
    SdfRaycastParams::SdfRaycastParams() {}

    void SdfRaycastParams::setTransformation(const glm::dmat4& transformation)
    {
        modelMatrix    = glm::mat4(transformation);
        invModelMatrix = glm::mat4(glm::inverse(transformation));
        normalMatrix   = glm::mat4(glm::inverseTranspose(glm::mat3(modelMatrix)));
    }
}

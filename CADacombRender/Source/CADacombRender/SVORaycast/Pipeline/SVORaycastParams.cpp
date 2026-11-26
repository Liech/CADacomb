#include "CADacombRender/SVORaycast/Pipeline/SVORaycastParams.h"

#include <glm/gtc/matrix_inverse.hpp>

namespace CADacomb::Render
{
    SVORaycastParams::SVORaycastParams() {}

    void SVORaycastParams::setTransformation(const glm::dmat4& transform)
    {
        modelMatrix    = glm::mat4(transform);
        invModelMatrix = glm::mat4(glm::inverse(transform));
        normalMatrix   = glm::mat4(glm::inverseTranspose(glm::mat3(modelMatrix)));
    }
}

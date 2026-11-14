#include "CADacomb/Util/Ray.h"

namespace CADacomb
{
    Ray::Ray() {}

    Ray::Ray(const glm::dvec3& rayOrigin, const glm::dvec3& rayDirection)
      : origin(rayOrigin)
      , direction(glm::normalize(rayDirection))
    {
    }

    Ray::~Ray() {}

    Ray Ray::fromScreen(const glm::vec2& screenPos, const glm::vec2& resolution, const glm::mat4& invViewMatrix, const glm::mat4& invProjectionMatrix)
    {
        if (resolution.x <= 0.0f || resolution.y <= 0.0f)
        {
            return Ray();
        }

        double ndcX = (2.0 * static_cast<double>(screenPos.x)) / static_cast<double>(resolution.x) - 1.0;
        double ndcY = (2.0 * static_cast<double>(screenPos.y)) / static_cast<double>(resolution.y) - 1.0;

        glm::dmat4 invViewProj = glm::dmat4(invViewMatrix) * glm::dmat4(invProjectionMatrix);

        glm::dvec4 nearClip(ndcX, ndcY, -1.0, 1.0);
        glm::dvec4 midClip(ndcX, ndcY, 0.0, 1.0);

        glm::dvec4 nearWorldH = invViewProj * nearClip;
        glm::dvec4 midWorldH  = invViewProj * midClip;

        if (glm::abs(nearWorldH.w) < 1e-12 || glm::abs(midWorldH.w) < 1e-12)
        {
            return Ray();
        }

        glm::dvec3 nearWorld = glm::dvec3(nearWorldH) / nearWorldH.w;
        glm::dvec3 midWorld  = glm::dvec3(midWorldH) / midWorldH.w;

        return Ray(nearWorld, midWorld - nearWorld);
    }

    const glm::dvec3& Ray::getOrigin() const
    {
        return origin;
    }

    const glm::dvec3& Ray::getDirection() const
    {
        return direction;
    }
}

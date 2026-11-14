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

        float ndcX = (2.0f * screenPos.x) / resolution.x - 1.0f;
        float ndcY = (2.0f * screenPos.y) / resolution.y - 1.0f;

        glm::mat4 invViewProj = invViewMatrix * invProjectionMatrix;

        glm::vec4 nearClip(ndcX, ndcY, 0.0f, 1.0f);
        glm::vec4 farClip(ndcX, ndcY, 1.0f, 1.0f);

        glm::vec4 nearWorldH = invViewProj * nearClip;
        glm::vec4 farWorldH  = invViewProj * farClip;

        if (glm::abs(nearWorldH.w) < 1e-6f || glm::abs(farWorldH.w) < 1e-6f)
        {
            return Ray();
        }

        glm::dvec3 nearWorld = glm::dvec3(nearWorldH) / static_cast<double>(nearWorldH.w);
        glm::dvec3 farWorld  = glm::dvec3(farWorldH) / static_cast<double>(farWorldH.w);

        return Ray(nearWorld, farWorld - nearWorld);
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

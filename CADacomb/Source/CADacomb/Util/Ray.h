#pragma once

#include <glm/glm.hpp>

namespace CADacomb
{
    // World-space ray used for intersection and picking queries.
    class Ray
    {
      public:
        Ray();
        Ray(const glm::dvec3& origin, const glm::dvec3& direction);
        virtual ~Ray();

        static Ray fromScreen(const glm::vec2& screenPos, const glm::vec2& resolution, const glm::mat4& invViewMatrix, const glm::mat4& invProjectionMatrix);

        const glm::dvec3& getOrigin() const;
        const glm::dvec3& getDirection() const;

      private:
        glm::dvec3 origin    = glm::dvec3(0.0);
        glm::dvec3 direction = glm::dvec3(0.0, 0.0, -1.0);
    };
}

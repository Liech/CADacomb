#pragma once

#include <glm/glm.hpp>

namespace CADacomb
{
    class AABB3
    {
      public:
        AABB3();
        AABB3(const glm::dvec3& minPoint, const glm::dvec3& maxPoint);
        virtual ~AABB3();

        const glm::dvec3& getMin() const;
        const glm::dvec3& getMax() const;
        void              setMin(const glm::dvec3& minPoint);
        void              setMax(const glm::dvec3& maxPoint);

        glm::dvec3 getSize() const;
        glm::dvec3 getCenter() const;

        bool contains(const glm::dvec3& point) const;
        bool contains(const AABB3& other) const;
        bool intersects(const AABB3& other) const;

        void expand(const glm::dvec3& point);
        void expand(const AABB3& other);

        glm::dvec3 closestPoint(const glm::dvec3& point) const;
        double     distanceTo(const glm::dvec3& point) const;
        double     squaredDistanceTo(const glm::dvec3& point) const;

      private:
        glm::dvec3 minPoint{ 0.0 };
        glm::dvec3 maxPoint{ 0.0 };
    };
}

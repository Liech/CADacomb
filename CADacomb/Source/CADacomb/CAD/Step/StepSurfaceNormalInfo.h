#pragma once

#include <glm/glm.hpp>
#include <string>

namespace CADacomb::CAD
{
    class StepSurfaceNormalInfo
    {
      public:
        StepSurfaceNormalInfo();
        virtual ~StepSurfaceNormalInfo();

        const std::string& getKind() const;
        void               setKind(const std::string& kindInput);
        const glm::dvec3&  getOrigin() const;
        void               setOrigin(const glm::dvec3& originInput);
        const glm::dvec3&  getZAxis() const;
        void               setZAxis(const glm::dvec3& zAxisInput);
        double             getRadius() const;
        void               setRadius(double radiusInput);
        double             getSemiAngleRad() const;
        void               setSemiAngleRad(double semiAngleRadInput);
        double             getMajorRadius() const;
        void               setMajorRadius(double majorRadiusInput);

      private:
        std::string kind;
        glm::dvec3  origin;
        glm::dvec3  zAxis;
        double      radius;
        double      semiAngleRad;
        double      majorRadius;
    };
}

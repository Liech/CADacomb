#include "StepSurfaceNormalInfo.h"

namespace CADacomb::CAD
{
    StepSurfaceNormalInfo::StepSurfaceNormalInfo()
      : kind("")
      , origin(0.0, 0.0, 0.0)
      , zAxis(0.0, 0.0, 1.0)
      , radius(1.0)
      , semiAngleRad(0.0)
      , majorRadius(1.0)
    {
    }

    StepSurfaceNormalInfo::~StepSurfaceNormalInfo() {}

    const std::string& StepSurfaceNormalInfo::getKind() const
    {
        return kind;
    }

    void StepSurfaceNormalInfo::setKind(const std::string& kindInput)
    {
        kind = kindInput;
    }

    const glm::dvec3& StepSurfaceNormalInfo::getOrigin() const
    {
        return origin;
    }

    void StepSurfaceNormalInfo::setOrigin(const glm::dvec3& originInput)
    {
        origin = originInput;
    }

    const glm::dvec3& StepSurfaceNormalInfo::getZAxis() const
    {
        return zAxis;
    }

    void StepSurfaceNormalInfo::setZAxis(const glm::dvec3& zAxisInput)
    {
        zAxis = zAxisInput;
    }

    double StepSurfaceNormalInfo::getRadius() const
    {
        return radius;
    }

    void StepSurfaceNormalInfo::setRadius(double radiusInput)
    {
        radius = radiusInput;
    }

    double StepSurfaceNormalInfo::getSemiAngleRad() const
    {
        return semiAngleRad;
    }

    void StepSurfaceNormalInfo::setSemiAngleRad(double semiAngleRadInput)
    {
        semiAngleRad = semiAngleRadInput;
    }

    double StepSurfaceNormalInfo::getMajorRadius() const
    {
        return majorRadius;
    }

    void StepSurfaceNormalInfo::setMajorRadius(double majorRadiusInput)
    {
        majorRadius = majorRadiusInput;
    }
}

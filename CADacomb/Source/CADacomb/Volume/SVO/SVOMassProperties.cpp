#include "CADacomb/Volume/SVO/SVOMassProperties.h"

namespace CADacomb
{
    SVOMassProperties::SVOMassProperties() {}

    SVOMassProperties::SVOMassProperties(double massInput, const glm::dvec3& centerOfMassInput, const glm::dmat3& inertiaTensorInput)
      : mass(massInput)
      , centerOfMass(centerOfMassInput)
      , inertiaTensor(inertiaTensorInput)
    {
    }

    SVOMassProperties::~SVOMassProperties() {}

    SVOMassProperties SVOMassProperties::compute(const SVO<bool>& svo, const AABB3& localBounds, double density)
    {
        uint32_t dimension = svo.getDimension();
        if (dimension == 0 || density <= 0.0)
        {
            return SVOMassProperties(1.0, localBounds.getCenter(), glm::dmat3(1.0));
        }

        glm::dvec3 voxelScale = localBounds.getSize() / static_cast<double>(dimension);
        double     totalMass  = 0.0;
        glm::dvec3 weightedCenter(0.0);

        svo.traverseLeaves(
          [&](const SVONode<bool>& node, const glm::ivec3& nodeOffset, uint32_t nodeDim, int)
          {
              if (!node.value)
              {
                  return;
              }
              glm::dvec3 boxMin    = localBounds.getMin() + glm::dvec3(nodeOffset) * voxelScale;
              glm::dvec3 boxSize   = glm::dvec3(static_cast<double>(nodeDim)) * voxelScale;
              glm::dvec3 boxCenter = boxMin + boxSize * 0.5;
              double     boxVolume = boxSize.x * boxSize.y * boxSize.z;
              double     boxMass   = boxVolume * density;

              totalMass += boxMass;
              weightedCenter += boxMass * boxCenter;
          });

        if (totalMass <= 0.0)
        {
            return SVOMassProperties(1.0, localBounds.getCenter(), glm::dmat3(1.0));
        }

        glm::dvec3 com = weightedCenter / totalMass;
        glm::dmat3 inertia(0.0);

        svo.traverseLeaves(
          [&](const SVONode<bool>& node, const glm::ivec3& nodeOffset, uint32_t nodeDim, int)
          {
              if (!node.value)
              {
                  return;
              }
              glm::dvec3 boxMin    = localBounds.getMin() + glm::dvec3(nodeOffset) * voxelScale;
              glm::dvec3 boxSize   = glm::dvec3(static_cast<double>(nodeDim)) * voxelScale;
              glm::dvec3 boxCenter = boxMin + boxSize * 0.5;
              double     boxVolume = boxSize.x * boxSize.y * boxSize.z;
              double     boxMass   = boxVolume * density;

              double ixx = (boxMass / 12.0) * (boxSize.y * boxSize.y + boxSize.z * boxSize.z);
              double iyy = (boxMass / 12.0) * (boxSize.x * boxSize.x + boxSize.z * boxSize.z);
              double izz = (boxMass / 12.0) * (boxSize.x * boxSize.x + boxSize.y * boxSize.y);

              glm::dvec3 r   = boxCenter - com;
              double     rSq = glm::dot(r, r);

              inertia[0][0] += ixx + boxMass * (rSq - r.x * r.x);
              inertia[1][1] += iyy + boxMass * (rSq - r.y * r.y);
              inertia[2][2] += izz + boxMass * (rSq - r.z * r.z);

              inertia[0][1] -= boxMass * r.x * r.y;
              inertia[1][0] -= boxMass * r.x * r.y;
              inertia[0][2] -= boxMass * r.x * r.z;
              inertia[2][0] -= boxMass * r.x * r.z;
              inertia[1][2] -= boxMass * r.y * r.z;
              inertia[2][1] -= boxMass * r.y * r.z;
          });

        return SVOMassProperties(totalMass, com, inertia);
    }

    double SVOMassProperties::getMass() const
    {
        return mass;
    }

    const glm::dvec3& SVOMassProperties::getCenterOfMass() const
    {
        return centerOfMass;
    }

    const glm::dmat3& SVOMassProperties::getInertiaTensor() const
    {
        return inertiaTensor;
    }
}

#ifdef ISTESTPROJECT
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

TEST_CASE("SVOMassProperties/FullCubeMatchesAnalyticalBoxInertia")
{
    CADacomb::SVO<bool> svo(8, true);
    CADacomb::AABB3     bounds(glm::dvec3(-1.0, -2.0, -3.0), glm::dvec3(1.0, 2.0, 3.0));
    double              density = 10.0;

    CADacomb::SVOMassProperties props        = CADacomb::SVOMassProperties::compute(svo, bounds, density);
    double                      expectedMass = 2.0 * 4.0 * 6.0 * density;

    CHECK(props.getMass() == Catch::Approx(expectedMass));
    CHECK(props.getCenterOfMass().x == Catch::Approx(0.0));
    CHECK(props.getCenterOfMass().y == Catch::Approx(0.0));
    CHECK(props.getCenterOfMass().z == Catch::Approx(0.0));

    CHECK(props.getInertiaTensor()[0][0] == Catch::Approx((expectedMass / 12.0) * (16.0 + 36.0)));
    CHECK(props.getInertiaTensor()[1][1] == Catch::Approx((expectedMass / 12.0) * (4.0 + 36.0)));
    CHECK(props.getInertiaTensor()[2][2] == Catch::Approx((expectedMass / 12.0) * (4.0 + 16.0)));
}
#endif

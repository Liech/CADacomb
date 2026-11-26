#pragma once

#include <glm/glm.hpp>
#include <vector>

namespace CADacomb
{
    class Triangulation;
}

namespace CADacomb::CAD
{
    // Fits a sphere to a triangle cluster and measures its maximum radial deviation.
    class SphereSurfaceFit
    {
      public:
        SphereSurfaceFit();
        virtual ~SphereSurfaceFit();

        static SphereSurfaceFit fit(const Triangulation& mesh, const std::vector<size_t>& clusterTriangles);

        bool              isValid() const;
        const glm::dvec3& getCenter() const;
        double            getRadius() const;
        double            getMaxDeviation() const;

      private:
        static bool solve4x4(double a[4][4], double b[4], double x[4]);

        bool       valid        = false;
        glm::dvec3 center       = glm::dvec3(0.0);
        double     radius       = 0.0;
        double     maxDeviation = 0.0;
    };
}

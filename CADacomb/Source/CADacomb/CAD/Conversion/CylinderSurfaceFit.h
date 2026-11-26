#pragma once

#include <glm/glm.hpp>
#include <vector>

namespace CADacomb
{
    class Triangulation;
}

namespace CADacomb::CAD
{
    // Fits a circular cylinder to a triangle cluster and measures its maximum radial deviation.
    class CylinderSurfaceFit
    {
      public:
        CylinderSurfaceFit();
        virtual ~CylinderSurfaceFit();

        static CylinderSurfaceFit fit(const Triangulation& mesh, const std::vector<size_t>& clusterTriangles);

        bool              isValid() const;
        const glm::dvec3& getOrigin() const;
        const glm::dvec3& getAxis() const;
        double            getRadius() const;
        double            getHeight() const;
        double            getMaxDeviation() const;

      private:
        static glm::dvec3 computeSmallestEigenvector(const double cov[3][3]);
        static bool       solve3x3(double a[3][3], double b[3], double x[3]);

        bool       valid        = false;
        glm::dvec3 origin       = glm::dvec3(0.0);
        glm::dvec3 axis         = glm::dvec3(0.0, 0.0, 1.0);
        double     radius       = 0.0;
        double     height       = 0.0;
        double     maxDeviation = 0.0;
    };
}

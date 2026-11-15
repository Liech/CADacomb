#pragma once

#include <glm/glm.hpp>
#include <vector>

namespace CADacomb
{
    class Triangulation;
}

namespace CADacomb::CAD
{
    // Fits a circular cone frustum to a triangle cluster and measures its maximum normal deviation.
    class ConeSurfaceFit
    {
      public:
        ConeSurfaceFit();
        virtual ~ConeSurfaceFit();

        static ConeSurfaceFit fit(const Triangulation& mesh, const std::vector<size_t>& clusterTriangles);

        bool              isValid() const;
        const glm::dvec3& getOrigin() const;
        const glm::dvec3& getAxis() const;
        double            getBotRadius() const;
        double            getTopRadius() const;
        double            getHeight() const;
        double            getMaxDeviation() const;

      private:
        static glm::dvec3 computeSmallestEigenvector(const double cov[3][3]);
        static bool       solve4x4(double a[4][4], double b[4], double x[4]);

        bool       valid        = false;
        glm::dvec3 origin       = glm::dvec3(0.0);
        glm::dvec3 axis         = glm::dvec3(0.0, 0.0, 1.0);
        double     botRadius    = 0.0;
        double     topRadius    = 0.0;
        double     height       = 0.0;
        double     maxDeviation = 0.0;
    };
}

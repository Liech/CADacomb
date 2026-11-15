#pragma once

#include <glm/glm.hpp>
#include <vector>

namespace CADacomb
{
    class Triangulation;
}

namespace CADacomb::CAD
{
    // Fits a regularized uniform bicubic B-spline surface to a triangle cluster.
    class NurbsSurfaceFit
    {
      public:
        NurbsSurfaceFit();
        virtual ~NurbsSurfaceFit();

        static NurbsSurfaceFit fit(const Triangulation& mesh, const std::vector<size_t>& clusterTriangles, size_t numRows = 6, size_t numCols = 6, double smoothingWeight = 1e-3);

        bool                           isValid() const;
        size_t                         getNumRows() const;
        size_t                         getNumCols() const;
        const std::vector<glm::dvec3>& getControlPoints() const;
        double                         getMaxDeviation() const;

      private:
        static std::vector<double> buildClampedUniformKnots(size_t count, size_t degree);
        static std::vector<double> evaluateBasis(const std::vector<double>& knots, size_t count, size_t degree, double t);
        static bool                solveLinearSystem(std::vector<double>& matrix, std::vector<glm::dvec3>& rhs, size_t n);

        bool                    valid        = false;
        size_t                  numRows      = 0;
        size_t                  numCols      = 0;
        std::vector<glm::dvec3> controlPoints;
        double                  maxDeviation = 0.0;
    };
}

#pragma once

#include <glm/glm.hpp>
#include <vector>

namespace CADacomb
{
    class Triangulation;
}

namespace CADacomb::CAD
{
    // Fits a best-fit plane to a triangle cluster and measures its maximum vertex deviation.
    class PlanarSurfaceFit
    {
      public:
        PlanarSurfaceFit();
        virtual ~PlanarSurfaceFit();

        static PlanarSurfaceFit fit(const Triangulation& mesh, const std::vector<size_t>& clusterTriangles);

        bool              isValid() const;
        const glm::dvec3& getOrigin() const;
        const glm::dvec3& getNormal() const;
        double            getMaxDeviation() const;

      private:
        bool       valid        = false;
        glm::dvec3 origin       = glm::dvec3(0.0);
        glm::dvec3 normal       = glm::dvec3(0.0, 0.0, 1.0);
        double     maxDeviation = 0.0;
    };
}

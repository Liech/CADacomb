#include "PlanarSurfaceFit.h"

#include "CADacomb/Triangle/Triangulation.h"

#include <algorithm>
#include <cmath>
#include <unordered_set>

namespace CADacomb::CAD
{
    PlanarSurfaceFit::PlanarSurfaceFit() {}

    PlanarSurfaceFit::~PlanarSurfaceFit() {}

    PlanarSurfaceFit PlanarSurfaceFit::fit(const Triangulation& mesh, const std::vector<size_t>& clusterTriangles)
    {
        PlanarSurfaceFit result;
        if (clusterTriangles.empty())
        {
            return result;
        }

        glm::dvec3 weightedNormal(0.0);
        glm::dvec3 weightedCentroid(0.0);
        double     totalArea = 0.0;

        std::unordered_set<size_t> uniqueVertices;
        uniqueVertices.reserve(clusterTriangles.size() * 3);

        for (size_t i = 0; i < clusterTriangles.size(); i++)
        {
            size_t triIndex = clusterTriangles[i];
            size_t i0       = mesh.indices[triIndex * 3 + 0];
            size_t i1       = mesh.indices[triIndex * 3 + 1];
            size_t i2       = mesh.indices[triIndex * 3 + 2];

            uniqueVertices.insert(i0);
            uniqueVertices.insert(i1);
            uniqueVertices.insert(i2);

            const glm::dvec3& p0    = mesh.vertices[i0];
            const glm::dvec3& p1    = mesh.vertices[i1];
            const glm::dvec3& p2    = mesh.vertices[i2];
            glm::dvec3        cross = glm::cross(p1 - p0, p2 - p0);
            double            area2 = glm::length(cross);

            if (area2 > 1e-14)
            {
                double area = 0.5 * area2;
                weightedNormal += cross * 0.5;
                weightedCentroid += ((p0 + p1 + p2) / 3.0) * area;
                totalArea += area;
            }
        }

        double normalLength = glm::length(weightedNormal);
        if (normalLength < 1e-12 || totalArea < 1e-14)
        {
            return result;
        }

        result.valid  = true;
        result.origin = weightedCentroid / totalArea;
        result.normal = weightedNormal / normalLength;

        double maxDev = 0.0;
        for (std::unordered_set<size_t>::const_iterator it = uniqueVertices.begin(); it != uniqueVertices.end(); it++)
        {
            double dist = std::abs(glm::dot(mesh.vertices[*it] - result.origin, result.normal));
            maxDev      = std::max(maxDev, dist);
        }
        result.maxDeviation = maxDev;

        return result;
    }

    bool PlanarSurfaceFit::isValid() const
    {
        return valid;
    }

    const glm::dvec3& PlanarSurfaceFit::getOrigin() const
    {
        return origin;
    }

    const glm::dvec3& PlanarSurfaceFit::getNormal() const
    {
        return normal;
    }

    double PlanarSurfaceFit::getMaxDeviation() const
    {
        return maxDeviation;
    }
}

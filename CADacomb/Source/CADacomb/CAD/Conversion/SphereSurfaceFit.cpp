#include "SphereSurfaceFit.h"

#include "CADacomb/Triangle/Triangulation.h"

#include <algorithm>
#include <cmath>
#include <unordered_set>

namespace CADacomb::CAD
{
    SphereSurfaceFit::SphereSurfaceFit() {}

    SphereSurfaceFit::~SphereSurfaceFit() {}

    bool SphereSurfaceFit::solve4x4(double a[4][4], double b[4], double x[4])
    {
        for (int col = 0; col < 4; col++)
        {
            int    pivot  = col;
            double maxVal = std::abs(a[col][col]);
            for (int row = col + 1; row < 4; row++)
            {
                if (std::abs(a[row][col]) > maxVal)
                {
                    maxVal = std::abs(a[row][col]);
                    pivot  = row;
                }
            }
            if (maxVal < 1e-14)
            {
                return false;
            }
            if (pivot != col)
            {
                for (int k = col; k < 4; k++)
                {
                    std::swap(a[col][k], a[pivot][k]);
                }
                std::swap(b[col], b[pivot]);
            }
            for (int row = col + 1; row < 4; row++)
            {
                double f = a[row][col] / a[col][col];
                for (int k = col; k < 4; k++)
                {
                    a[row][k] -= f * a[col][k];
                }
                b[row] -= f * b[col];
            }
        }
        for (int row = 3; row >= 0; row--)
        {
            double s = b[row];
            for (int k = row + 1; k < 4; k++)
            {
                s -= a[row][k] * x[k];
            }
            x[row] = s / a[row][row];
        }
        return true;
    }

    SphereSurfaceFit SphereSurfaceFit::fit(const Triangulation& mesh, const std::vector<size_t>& clusterTriangles)
    {
        SphereSurfaceFit result;
        if (clusterTriangles.size() < 4)
        {
            return result;
        }

        std::unordered_set<size_t> uniqueIndices;
        for (size_t triIdx : clusterTriangles)
        {
            uniqueIndices.insert(mesh.indices[triIdx * 3 + 0]);
            uniqueIndices.insert(mesh.indices[triIdx * 3 + 1]);
            uniqueIndices.insert(mesh.indices[triIdx * 3 + 2]);
        }
        if (uniqueIndices.size() < 4)
        {
            return result;
        }

        std::vector<glm::dvec3> pts;
        pts.reserve(uniqueIndices.size());
        glm::dvec3 mean(0.0);
        for (size_t idx : uniqueIndices)
        {
            pts.push_back(mesh.vertices[idx]);
            mean += mesh.vertices[idx];
        }
        mean /= static_cast<double>(pts.size());

        double m[4][4] = {};
        double rhs[4]  = {};
        for (const glm::dvec3& pt : pts)
        {
            glm::dvec3 p   = pt - mean;
            double     r2  = glm::dot(p, p);
            double     row[4] = { 2.0 * p.x, 2.0 * p.y, 2.0 * p.z, 1.0 };
            for (int r = 0; r < 4; r++)
            {
                for (int c = 0; c < 4; c++)
                {
                    m[r][c] += row[r] * row[c];
                }
                rhs[r] += row[r] * r2;
            }
        }

        double sol[4] = {};
        if (!solve4x4(m, rhs, sol))
        {
            return result;
        }

        glm::dvec3 localCenter(sol[0], sol[1], sol[2]);
        double     radSq = glm::dot(localCenter, localCenter) + sol[3];
        if (radSq <= 1e-12)
        {
            return result;
        }

        result.center = mean + localCenter;
        result.radius = std::sqrt(radSq);

        double maxDev = 0.0;
        for (const glm::dvec3& pt : pts)
        {
            maxDev = std::max(maxDev, std::abs(glm::length(pt - result.center) - result.radius));
        }

        // Verify consistent radial orientation of triangle normals to reject flat/saddle patches.
        int signSum = 0;
        for (size_t triIdx : clusterTriangles)
        {
            const glm::dvec3& p0  = mesh.vertices[mesh.indices[triIdx * 3 + 0]];
            const glm::dvec3& p1  = mesh.vertices[mesh.indices[triIdx * 3 + 1]];
            const glm::dvec3& p2  = mesh.vertices[mesh.indices[triIdx * 3 + 2]];
            glm::dvec3        n   = glm::cross(p1 - p0, p2 - p0);
            double            len = glm::length(n);
            if (len < 1e-12)
            {
                continue;
            }
            glm::dvec3 radial = (p0 + p1 + p2) / 3.0 - result.center;
            double     rLen   = glm::length(radial);
            if (rLen < 1e-12)
            {
                return result;
            }
            double align = glm::dot(n / len, radial / rLen);
            if (std::abs(align) < 0.5)
            {
                return result;
            }
            signSum += (align > 0.0) ? 1 : -1;
        }
        if (std::abs(signSum) != static_cast<int>(clusterTriangles.size()))
        {
            return result;
        }

        result.maxDeviation = maxDev;
        result.valid        = true;
        return result;
    }

    bool SphereSurfaceFit::isValid() const
    {
        return valid;
    }

    const glm::dvec3& SphereSurfaceFit::getCenter() const
    {
        return center;
    }

    double SphereSurfaceFit::getRadius() const
    {
        return radius;
    }

    double SphereSurfaceFit::getMaxDeviation() const
    {
        return maxDeviation;
    }
}

#ifdef ISTESTPROJECT
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

TEST_CASE("SphereSurfaceFit/FitsSpherePatch")
{
    CADacomb::Triangulation mesh;
    const glm::dvec3        center(1.0, -2.0, 0.5);
    const double            radius = 2.5;
    const size_t            nPhi   = 8;
    const size_t            nTheta = 12;

    for (size_t i = 0; i <= nPhi; i++)
    {
        double phi = 0.2 + 1.0 * static_cast<double>(i) / static_cast<double>(nPhi);
        for (size_t j = 0; j < nTheta; j++)
        {
            double theta = 6.28318530718 * static_cast<double>(j) / static_cast<double>(nTheta);
            mesh.vertices.push_back(center + radius * glm::dvec3(std::sin(phi) * std::cos(theta), std::sin(phi) * std::sin(theta), std::cos(phi)));
        }
    }

    std::vector<size_t> cluster;
    for (size_t i = 0; i < nPhi; i++)
    {
        for (size_t j = 0; j < nTheta; j++)
        {
            size_t jNext = (j + 1) % nTheta;
            size_t a     = i * nTheta + j;
            size_t b     = i * nTheta + jNext;
            size_t c     = (i + 1) * nTheta + j;
            size_t d     = (i + 1) * nTheta + jNext;
            cluster.push_back(mesh.indices.size() / 3);
            mesh.indices.push_back(a);
            mesh.indices.push_back(c);
            mesh.indices.push_back(b);
            cluster.push_back(mesh.indices.size() / 3);
            mesh.indices.push_back(b);
            mesh.indices.push_back(c);
            mesh.indices.push_back(d);
        }
    }

    CADacomb::CAD::SphereSurfaceFit fit = CADacomb::CAD::SphereSurfaceFit::fit(mesh, cluster);
    REQUIRE(fit.isValid());
    REQUIRE(fit.getRadius() == Catch::Approx(radius).margin(1e-4));
    REQUIRE(fit.getCenter().x == Catch::Approx(center.x).margin(1e-4));
    REQUIRE(fit.getCenter().y == Catch::Approx(center.y).margin(1e-4));
    REQUIRE(fit.getCenter().z == Catch::Approx(center.z).margin(1e-4));
    REQUIRE(fit.getMaxDeviation() < 1e-4);
}
#endif

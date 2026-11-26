#include "CylinderSurfaceFit.h"

#include "CADacomb/Triangle/Triangulation.h"

#include <algorithm>
#include <cmath>
#include <unordered_set>

namespace CADacomb::CAD
{
    CylinderSurfaceFit::CylinderSurfaceFit() {}

    CylinderSurfaceFit::~CylinderSurfaceFit() {}

    glm::dvec3 CylinderSurfaceFit::computeSmallestEigenvector(const double cov[3][3])
    {
        double a[3][3] = {
            { cov[0][0], cov[0][1], cov[0][2] },
            { cov[1][0], cov[1][1], cov[1][2] },
            { cov[2][0], cov[2][1], cov[2][2] }
        };
        double v[3][3] = {
            { 1.0, 0.0, 0.0 },
            { 0.0, 1.0, 0.0 },
            { 0.0, 0.0, 1.0 }
        };

        for (int iter = 0; iter < 30; iter++)
        {
            int    p      = 0;
            int    q      = 1;
            double maxOff = std::abs(a[0][1]);
            if (std::abs(a[0][2]) > maxOff)
            {
                p      = 0;
                q      = 2;
                maxOff = std::abs(a[0][2]);
            }
            if (std::abs(a[1][2]) > maxOff)
            {
                p      = 1;
                q      = 2;
                maxOff = std::abs(a[1][2]);
            }
            if (maxOff < 1e-14)
            {
                break;
            }

            double app = a[p][p];
            double aqq = a[q][q];
            double apq = a[p][q];
            double tau = (aqq - app) / (2.0 * apq);
            double t   = (tau >= 0.0) ? (1.0 / (tau + std::sqrt(1.0 + tau * tau))) : (-1.0 / (-tau + std::sqrt(1.0 + tau * tau)));
            double c   = 1.0 / std::sqrt(1.0 + t * t);
            double s   = t * c;

            a[p][p] = app - t * apq;
            a[q][q] = aqq + t * apq;
            a[p][q] = 0.0;
            a[q][p] = 0.0;

            for (int r = 0; r < 3; r++)
            {
                if (r != p && r != q)
                {
                    double arp = a[r][p];
                    double arq = a[r][q];
                    a[r][p] = a[p][r] = c * arp - s * arq;
                    a[r][q] = a[q][r] = s * arp + c * arq;
                }
            }

            for (int r = 0; r < 3; r++)
            {
                double vrp = v[r][p];
                double vrq = v[r][q];
                v[r][p]    = c * vrp - s * vrq;
                v[r][q]    = s * vrp + c * vrq;
            }
        }

        int minCol = 0;
        if (a[1][1] < a[minCol][minCol])
        {
            minCol = 1;
        }
        if (a[2][2] < a[minCol][minCol])
        {
            minCol = 2;
        }

        glm::dvec3 axisVec(v[0][minCol], v[1][minCol], v[2][minCol]);
        double     len = glm::length(axisVec);
        return (len > 1e-12) ? (axisVec / len) : glm::dvec3(0.0, 0.0, 1.0);
    }

    bool CylinderSurfaceFit::solve3x3(double a[3][3], double b[3], double x[3])
    {
        for (int col = 0; col < 3; col++)
        {
            int    pivot  = col;
            double maxVal = std::abs(a[col][col]);
            for (int row = col + 1; row < 3; row++)
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
                for (int k = col; k < 3; k++)
                {
                    std::swap(a[col][k], a[pivot][k]);
                }
                std::swap(b[col], b[pivot]);
            }
            for (int row = col + 1; row < 3; row++)
            {
                double f = a[row][col] / a[col][col];
                for (int k = col; k < 3; k++)
                {
                    a[row][k] -= f * a[col][k];
                }
                b[row] -= f * b[col];
            }
        }

        for (int row = 2; row >= 0; row--)
        {
            double sum = b[row];
            for (int k = row + 1; k < 3; k++)
            {
                sum -= a[row][k] * x[k];
            }
            x[row] = sum / a[row][row];
        }
        return true;
    }

    CylinderSurfaceFit CylinderSurfaceFit::fit(const Triangulation& mesh, const std::vector<size_t>& clusterTriangles)
    {
        CylinderSurfaceFit result;
        if (clusterTriangles.size() < 4)
        {
            return result;
        }

        std::unordered_set<size_t> uniqueIndices;
        double                     cov[3][3] = {
            { 0.0, 0.0, 0.0 },
            { 0.0, 0.0, 0.0 },
            { 0.0, 0.0, 0.0 }
        };

        for (size_t triIdx : clusterTriangles)
        {
            size_t i0 = mesh.indices[triIdx * 3 + 0];
            size_t i1 = mesh.indices[triIdx * 3 + 1];
            size_t i2 = mesh.indices[triIdx * 3 + 2];
            uniqueIndices.insert(i0);
            uniqueIndices.insert(i1);
            uniqueIndices.insert(i2);

            glm::dvec3 crossVec = glm::cross(mesh.vertices[i1] - mesh.vertices[i0], mesh.vertices[i2] - mesh.vertices[i0]);
            double     area2    = glm::length(crossVec);
            if (area2 < 1e-12)
            {
                continue;
            }
            glm::dvec3 n = crossVec / area2;
            for (int r = 0; r < 3; r++)
            {
                for (int c = 0; c < 3; c++)
                {
                    cov[r][c] += area2 * n[r] * n[c];
                }
            }
        }

        if (uniqueIndices.size() < 6)
        {
            return result;
        }

        glm::dvec3 axisVec = computeSmallestEigenvector(cov);

        std::vector<glm::dvec3> points;
        points.reserve(uniqueIndices.size());
        glm::dvec3 centroid(0.0);
        for (size_t vIdx : uniqueIndices)
        {
            points.push_back(mesh.vertices[vIdx]);
            centroid += mesh.vertices[vIdx];
        }
        centroid /= static_cast<double>(points.size());

        glm::dvec3 fallback = (std::abs(axisVec.x) < 0.9) ? glm::dvec3(1.0, 0.0, 0.0) : glm::dvec3(0.0, 1.0, 0.0);
        glm::dvec3 uAxis    = glm::normalize(glm::cross(axisVec, fallback));
        glm::dvec3 vAxis    = glm::normalize(glm::cross(axisVec, uAxis));

        double mat[3][3] = {
            { 0.0, 0.0, 0.0 },
            { 0.0, 0.0, 0.0 },
            { 0.0, 0.0, 0.0 }
        };
        double rhs[3] = { 0.0, 0.0, 0.0 };
        double tMin   = 1e30;
        double tMax   = -1e30;

        for (const glm::dvec3& pt : points)
        {
            glm::dvec3 rel = pt - centroid;
            double     t   = glm::dot(rel, axisVec);
            tMin           = std::min(tMin, t);
            tMax           = std::max(tMax, t);

            double x = glm::dot(rel, uAxis);
            double y = glm::dot(rel, vAxis);
            double z = x * x + y * y;

            double row[3] = { x, y, 1.0 };
            for (int r = 0; r < 3; r++)
            {
                rhs[r] += row[r] * z;
                for (int c = 0; c < 3; c++)
                {
                    mat[r][c] += row[r] * row[c];
                }
            }
        }

        double sol[3] = { 0.0, 0.0, 0.0 };
        if (!solve3x3(mat, rhs, sol))
        {
            return result;
        }

        double cx  = 0.5 * sol[0];
        double cy  = 0.5 * sol[1];
        double rad2 = cx * cx + cy * cy + sol[2];
        if (rad2 <= 1e-12)
        {
            return result;
        }

        double     rad          = std::sqrt(rad2);
        double     cylHeight    = std::max(tMax - tMin, 1e-6);
        glm::dvec3 cylOrigin    = centroid + cx * uAxis + cy * vAxis + tMin * axisVec;

        double maxDev = 0.0;
        for (const glm::dvec3& pt : points)
        {
            glm::dvec3 rel    = pt - cylOrigin;
            glm::dvec3 radial = rel - glm::dot(rel, axisVec) * axisVec;
            maxDev            = std::max(maxDev, std::abs(glm::length(radial) - rad));
        }

        result.valid        = true;
        result.origin       = cylOrigin;
        result.axis         = axisVec;
        result.radius       = rad;
        result.height       = cylHeight;
        result.maxDeviation = maxDev;
        return result;
    }

    bool CylinderSurfaceFit::isValid() const
    {
        return valid;
    }

    const glm::dvec3& CylinderSurfaceFit::getOrigin() const
    {
        return origin;
    }

    const glm::dvec3& CylinderSurfaceFit::getAxis() const
    {
        return axis;
    }

    double CylinderSurfaceFit::getRadius() const
    {
        return radius;
    }

    double CylinderSurfaceFit::getHeight() const
    {
        return height;
    }

    double CylinderSurfaceFit::getMaxDeviation() const
    {
        return maxDeviation;
    }
}

#ifdef ISTESTPROJECT
#include "CADacomb/Triangle/BaseShapeGenerator.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

TEST_CASE("CylinderSurfaceFit/CylinderSide")
{
    CADacomb::Triangulation mesh;
    const size_t            segs = 24;
    const double            r    = 2.5;
    const double            h    = 5.0;

    for (size_t i = 0; i < segs; i++)
    {
        double a = 2.0 * 3.14159265358979323846 * static_cast<double>(i) / static_cast<double>(segs);
        mesh.vertices.push_back(glm::dvec3(r * std::cos(a), r * std::sin(a), 0.0));
        mesh.vertices.push_back(glm::dvec3(r * std::cos(a), r * std::sin(a), h));
    }

    std::vector<size_t> cluster;
    for (size_t i = 0; i < segs; i++)
    {
        size_t next = (i + 1) % segs;
        size_t b0   = i * 2;
        size_t t0   = i * 2 + 1;
        size_t b1   = next * 2;
        size_t t1   = next * 2 + 1;
        cluster.push_back(mesh.indices.size() / 3);
        mesh.indices.push_back(b0);
        mesh.indices.push_back(b1);
        mesh.indices.push_back(t1);
        cluster.push_back(mesh.indices.size() / 3);
        mesh.indices.push_back(b0);
        mesh.indices.push_back(t1);
        mesh.indices.push_back(t0);
    }

    CADacomb::CAD::CylinderSurfaceFit fit = CADacomb::CAD::CylinderSurfaceFit::fit(mesh, cluster);
    REQUIRE(fit.isValid());
    REQUIRE(fit.getRadius() == Catch::Approx(r).margin(1e-3));
    REQUIRE(fit.getHeight() == Catch::Approx(h).margin(1e-3));
    REQUIRE(std::abs(glm::dot(fit.getAxis(), glm::dvec3(0.0, 0.0, 1.0))) == Catch::Approx(1.0).margin(1e-3));
    REQUIRE(fit.getMaxDeviation() < 1e-3);
}
#endif

#include "ConeSurfaceFit.h"

#include "CADacomb/Triangle/Triangulation.h"

#include <algorithm>
#include <cmath>
#include <unordered_set>

namespace CADacomb::CAD
{
    ConeSurfaceFit::ConeSurfaceFit() {}

    ConeSurfaceFit::~ConeSurfaceFit() {}

    glm::dvec3 ConeSurfaceFit::computeSmallestEigenvector(const double cov[3][3])
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

    bool ConeSurfaceFit::solve4x4(double a[4][4], double b[4], double x[4])
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

    ConeSurfaceFit ConeSurfaceFit::fit(const Triangulation& mesh, const std::vector<size_t>& clusterTriangles)
    {
        ConeSurfaceFit result;
        if (clusterTriangles.size() < 4)
        {
            return result;
        }

        std::vector<glm::dvec3> normals;
        normals.reserve(clusterTriangles.size());
        glm::dvec3 meanNormal(0.0);
        for (size_t triIdx : clusterTriangles)
        {
            const glm::dvec3& p0  = mesh.vertices[mesh.indices[triIdx * 3 + 0]];
            const glm::dvec3& p1  = mesh.vertices[mesh.indices[triIdx * 3 + 1]];
            const glm::dvec3& p2  = mesh.vertices[mesh.indices[triIdx * 3 + 2]];
            glm::dvec3        n   = glm::cross(p1 - p0, p2 - p0);
            double            len = glm::length(n);
            if (len > 1e-12)
            {
                glm::dvec3 unitN = n / len;
                normals.push_back(unitN);
                meanNormal += unitN;
            }
        }
        if (normals.size() < 4)
        {
            return result;
        }
        meanNormal /= static_cast<double>(normals.size());

        double cov[3][3] = {};
        for (const glm::dvec3& n : normals)
        {
            glm::dvec3 d = n - meanNormal;
            cov[0][0] += d.x * d.x;
            cov[0][1] += d.x * d.y;
            cov[0][2] += d.x * d.z;
            cov[1][1] += d.y * d.y;
            cov[1][2] += d.y * d.z;
            cov[2][2] += d.z * d.z;
        }
        cov[1][0] = cov[0][1];
        cov[2][0] = cov[0][2];
        cov[2][1] = cov[1][2];

        glm::dvec3 axisDir  = computeSmallestEigenvector(cov);
        glm::dvec3 fallback = (std::abs(axisDir.x) < 0.9) ? glm::dvec3(1.0, 0.0, 0.0) : glm::dvec3(0.0, 1.0, 0.0);
        glm::dvec3 basisU   = glm::normalize(glm::cross(axisDir, fallback));
        glm::dvec3 basisV   = glm::normalize(glm::cross(axisDir, basisU));

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
        glm::dvec3 meanPt(0.0);
        for (size_t idx : uniqueIndices)
        {
            pts.push_back(mesh.vertices[idx]);
            meanPt += mesh.vertices[idx];
        }
        meanPt /= static_cast<double>(pts.size());

        std::vector<double> xs(pts.size());
        std::vector<double> ys(pts.size());
        std::vector<double> hs(pts.size());
        double              hMin = 1e30;
        double              hMax = -1e30;
        double              rAvg = 0.0;
        for (size_t i = 0; i < pts.size(); i++)
        {
            glm::dvec3 rel = pts[i] - meanPt;
            xs[i]          = glm::dot(rel, basisU);
            ys[i]          = glm::dot(rel, basisV);
            hs[i]          = glm::dot(rel, axisDir);
            hMin           = std::min(hMin, hs[i]);
            hMax           = std::max(hMax, hs[i]);
            rAvg += std::hypot(xs[i], ys[i]);
        }
        rAvg /= static_cast<double>(pts.size());

        double height = hMax - hMin;
        if (height <= 1e-10)
        {
            return result;
        }

        double cu = 0.0;
        double cv = 0.0;
        double r0 = std::max(rAvg, 1e-4);
        double m  = 0.0;

        for (int iter = 0; iter < 15; iter++)
        {
            double jtJ[4][4] = {};
            double jtE[4]    = {};

            for (size_t i = 0; i < pts.size(); i++)
            {
                double dx = xs[i] - cu;
                double dy = ys[i] - cv;
                double s  = std::hypot(dx, dy);
                if (s < 1e-12)
                {
                    continue;
                }
                double e    = s - (r0 + m * hs[i]);
                double j[4] = { -dx / s, -dy / s, -1.0, -hs[i] };
                for (int r = 0; r < 4; r++)
                {
                    for (int c = 0; c < 4; c++)
                    {
                        jtJ[r][c] += j[r] * j[c];
                    }
                    jtE[r] -= j[r] * e;
                }
            }

            for (int d = 0; d < 4; d++)
            {
                jtJ[d][d] += 1e-9;
            }

            double delta[4] = {};
            if (!solve4x4(jtJ, jtE, delta))
            {
                return result;
            }

            cu += delta[0];
            cv += delta[1];
            r0 += delta[2];
            m += delta[3];

            if (delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2] + delta[3] * delta[3] < 1e-20)
            {
                break;
            }
        }

        double semiAngleDeg = std::atan(std::abs(m)) * 180.0 / 3.14159265358979323846;
        if (semiAngleDeg < 0.5 || semiAngleDeg > 89.5)
        {
            return result;
        }

        double rBot = r0 + m * hMin;
        double rTop = r0 + m * hMax;
        if (rBot < -1e-6 || rTop < -1e-6)
        {
            return result;
        }
        rBot = std::max(rBot, 0.0);
        rTop = std::max(rTop, 0.0);

        glm::dvec3 axisCenter = meanPt + cu * basisU + cv * basisV;
        double     cosAlpha   = 1.0 / std::sqrt(1.0 + m * m);
        double     maxDev     = 0.0;
        for (size_t i = 0; i < pts.size(); i++)
        {
            double s      = std::hypot(xs[i] - cu, ys[i] - cv);
            double rModel = r0 + m * hs[i];
            maxDev        = std::max(maxDev, std::abs(s - rModel) * cosAlpha);
        }

        if (rBot < rTop)
        {
            result.origin    = axisCenter + hMax * axisDir;
            result.axis      = -axisDir;
            result.botRadius = rTop;
            result.topRadius = rBot;
        }
        else
        {
            result.origin    = axisCenter + hMin * axisDir;
            result.axis      = axisDir;
            result.botRadius = rBot;
            result.topRadius = rTop;
        }
        result.height       = height;
        result.maxDeviation = maxDev;
        result.valid        = true;
        return result;
    }

    bool ConeSurfaceFit::isValid() const
    {
        return valid;
    }

    const glm::dvec3& ConeSurfaceFit::getOrigin() const
    {
        return origin;
    }

    const glm::dvec3& ConeSurfaceFit::getAxis() const
    {
        return axis;
    }

    double ConeSurfaceFit::getBotRadius() const
    {
        return botRadius;
    }

    double ConeSurfaceFit::getTopRadius() const
    {
        return topRadius;
    }

    double ConeSurfaceFit::getHeight() const
    {
        return height;
    }

    double ConeSurfaceFit::getMaxDeviation() const
    {
        return maxDeviation;
    }
}

#ifdef ISTESTPROJECT
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

TEST_CASE("ConeSurfaceFit/FitsConeFrustum")
{
    CADacomb::Triangulation mesh;
    const double            rBot   = 2.0;
    const double            rTop   = 1.0;
    const double            height = 3.0;
    const size_t            nTheta = 16;

    for (size_t i = 0; i <= 1; i++)
    {
        double h = static_cast<double>(i) * height;
        double r = rBot + (rTop - rBot) * static_cast<double>(i);
        for (size_t j = 0; j < nTheta; j++)
        {
            double theta = 6.28318530718 * static_cast<double>(j) / static_cast<double>(nTheta);
            mesh.vertices.push_back(glm::dvec3(r * std::cos(theta), r * std::sin(theta), h));
        }
    }

    std::vector<size_t> cluster;
    for (size_t j = 0; j < nTheta; j++)
    {
        size_t jNext = (j + 1) % nTheta;
        size_t a     = j;
        size_t b     = jNext;
        size_t c     = nTheta + j;
        size_t d     = nTheta + jNext;
        cluster.push_back(mesh.indices.size() / 3);
        mesh.indices.push_back(a);
        mesh.indices.push_back(b);
        mesh.indices.push_back(c);
        cluster.push_back(mesh.indices.size() / 3);
        mesh.indices.push_back(b);
        mesh.indices.push_back(d);
        mesh.indices.push_back(c);
    }

    CADacomb::CAD::ConeSurfaceFit fit = CADacomb::CAD::ConeSurfaceFit::fit(mesh, cluster);
    REQUIRE(fit.isValid());
    REQUIRE(fit.getBotRadius() == Catch::Approx(rBot).margin(1e-3));
    REQUIRE(fit.getTopRadius() == Catch::Approx(rTop).margin(1e-3));
    REQUIRE(fit.getHeight() == Catch::Approx(height).margin(1e-3));
    REQUIRE(fit.getMaxDeviation() < 1e-4);
}
#endif

#include "NurbsSurfaceFit.h"

#include "CADacomb/Triangle/Triangulation.h"

#include <algorithm>
#include <cmath>
#include <unordered_set>

namespace CADacomb::CAD
{
    NurbsSurfaceFit::NurbsSurfaceFit() {}

    NurbsSurfaceFit::~NurbsSurfaceFit() {}

    std::vector<double> NurbsSurfaceFit::buildClampedUniformKnots(size_t count, size_t degree)
    {
        size_t              knotCount = count + degree + 1;
        std::vector<double> knots(knotCount, 0.0);
        size_t              interiorCount = count - degree - 1;
        for (size_t i = 0; i < interiorCount; i++)
        {
            knots[degree + 1 + i] = static_cast<double>(i + 1) / static_cast<double>(interiorCount + 1);
        }
        for (size_t i = count; i < knotCount; i++)
        {
            knots[i] = 1.0;
        }
        return knots;
    }

    std::vector<double> NurbsSurfaceFit::evaluateBasis(const std::vector<double>& knots, size_t count, size_t degree, double t)
    {
        std::vector<double> basis(count, 0.0);
        if (t <= 0.0)
        {
            basis[0] = 1.0;
            return basis;
        }
        if (t >= 1.0)
        {
            basis[count - 1] = 1.0;
            return basis;
        }

        std::vector<double> prev(count + 1, 0.0);
        for (size_t i = 0; i < count; i++)
        {
            if (t >= knots[i] && t < knots[i + 1])
            {
                prev[i] = 1.0;
            }
        }

        for (size_t d = 1; d <= degree; d++)
        {
            std::vector<double> next(count + 1, 0.0);
            for (size_t i = 0; i < count; i++)
            {
                double denom1 = knots[i + d] - knots[i];
                double denom2 = knots[i + d + 1] - knots[i + 1];
                double term1  = (denom1 > 1e-14) ? ((t - knots[i]) / denom1) * prev[i] : 0.0;
                double term2  = (denom2 > 1e-14) ? ((knots[i + d + 1] - t) / denom2) * prev[i + 1] : 0.0;
                next[i]       = term1 + term2;
            }
            prev = std::move(next);
        }

        for (size_t i = 0; i < count; i++)
        {
            basis[i] = prev[i];
        }
        return basis;
    }

    bool NurbsSurfaceFit::solveLinearSystem(std::vector<double>& matrix, std::vector<glm::dvec3>& rhs, size_t n)
    {
        for (size_t col = 0; col < n; col++)
        {
            size_t pivotRow = col;
            double maxVal   = std::abs(matrix[col * n + col]);
            for (size_t row = col + 1; row < n; row++)
            {
                double val = std::abs(matrix[row * n + col]);
                if (val > maxVal)
                {
                    maxVal   = val;
                    pivotRow = row;
                }
            }

            if (maxVal < 1e-14)
            {
                return false;
            }

            if (pivotRow != col)
            {
                for (size_t k = col; k < n; k++)
                {
                    std::swap(matrix[col * n + k], matrix[pivotRow * n + k]);
                }
                std::swap(rhs[col], rhs[pivotRow]);
            }

            double diag = matrix[col * n + col];
            for (size_t row = col + 1; row < n; row++)
            {
                double factor = matrix[row * n + col] / diag;
                if (std::abs(factor) < 1e-16)
                {
                    continue;
                }
                for (size_t k = col; k < n; k++)
                {
                    matrix[row * n + k] -= factor * matrix[col * n + k];
                }
                rhs[row] -= factor * rhs[col];
            }
        }

        for (size_t idx = n; idx > 0; idx--)
        {
            size_t     row = idx - 1;
            glm::dvec3 sum = rhs[row];
            for (size_t k = row + 1; k < n; k++)
            {
                sum -= matrix[row * n + k] * rhs[k];
            }
            rhs[row] = sum / matrix[row * n + row];
        }
        return true;
    }

    NurbsSurfaceFit NurbsSurfaceFit::fit(const Triangulation& mesh, const std::vector<size_t>& clusterTriangles, size_t numRows, size_t numCols, double smoothingWeight)
    {
        NurbsSurfaceFit result;
        if (clusterTriangles.empty() || numRows < 4 || numCols < 4)
        {
            return result;
        }

        std::unordered_set<size_t> uniqueIndices;
        glm::dvec3                 avgNormal(0.0);
        for (size_t triIdx : clusterTriangles)
        {
            size_t i0 = mesh.indices[triIdx * 3 + 0];
            size_t i1 = mesh.indices[triIdx * 3 + 1];
            size_t i2 = mesh.indices[triIdx * 3 + 2];
            uniqueIndices.insert(i0);
            uniqueIndices.insert(i1);
            uniqueIndices.insert(i2);
            avgNormal += glm::cross(mesh.vertices[i1] - mesh.vertices[i0], mesh.vertices[i2] - mesh.vertices[i0]);
        }

        if (uniqueIndices.size() < 3)
        {
            return result;
        }

        std::vector<glm::dvec3> points;
        points.reserve(uniqueIndices.size());
        glm::dvec3 centroid(0.0);
        for (size_t vIdx : uniqueIndices)
        {
            points.push_back(mesh.vertices[vIdx]);
            centroid += mesh.vertices[vIdx];
        }
        centroid /= static_cast<double>(points.size());

        double normalLen = glm::length(avgNormal);
        if (normalLen < 1e-12)
        {
            avgNormal = glm::dvec3(0.0, 0.0, 1.0);
        }
        else
        {
            avgNormal /= normalLen;
        }

        for (size_t triIdx : clusterTriangles)
        {
            size_t     i0       = mesh.indices[triIdx * 3 + 0];
            size_t     i1       = mesh.indices[triIdx * 3 + 1];
            size_t     i2       = mesh.indices[triIdx * 3 + 2];
            glm::dvec3 triCross = glm::cross(mesh.vertices[i1] - mesh.vertices[i0], mesh.vertices[i2] - mesh.vertices[i0]);
            double     triLen   = glm::length(triCross);
            if (triLen > 1e-12 && glm::dot(triCross, avgNormal) <= 0.1 * triLen)
            {
                return result;
            }
        }

        glm::dvec3 fallbackAxis = (std::abs(avgNormal.x) < 0.9) ? glm::dvec3(1.0, 0.0, 0.0) : glm::dvec3(0.0, 1.0, 0.0);
        glm::dvec3 axisU        = glm::normalize(glm::cross(avgNormal, fallbackAxis));
        glm::dvec3 axisV        = glm::normalize(glm::cross(avgNormal, axisU));

        double uMin = 1e30;
        double uMax = -1e30;
        double vMin = 1e30;
        double vMax = -1e30;
        for (const glm::dvec3& pt : points)
        {
            glm::dvec3 rel = pt - centroid;
            double     u   = glm::dot(rel, axisU);
            double     v   = glm::dot(rel, axisV);
            uMin           = std::min(uMin, u);
            uMax           = std::max(uMax, u);
            vMin           = std::min(vMin, v);
            vMax           = std::max(vMax, v);
        }

        double uSpan = std::max(uMax - uMin, 1e-8);
        double vSpan = std::max(vMax - vMin, 1e-8);
        uMin -= 0.02 * uSpan;
        uMax += 0.02 * uSpan;
        vMin -= 0.02 * vSpan;
        vMax += 0.02 * vSpan;
        uSpan = uMax - uMin;
        vSpan = vMax - vMin;

        const size_t        degree = 3;
        std::vector<double> knotsU = buildClampedUniformKnots(numRows, degree);
        std::vector<double> knotsV = buildClampedUniformKnots(numCols, degree);

        size_t                  unknownCount = numRows * numCols;
        std::vector<double>     normalMatrix(unknownCount * unknownCount, 0.0);
        std::vector<glm::dvec3> rhs(unknownCount, glm::dvec3(0.0));

        std::vector<double> uParams(points.size(), 0.0);
        std::vector<double> vParams(points.size(), 0.0);

        for (size_t pIdx = 0; pIdx < points.size(); pIdx++)
        {
            glm::dvec3 rel = points[pIdx] - centroid;
            double     u   = (glm::dot(rel, axisU) - uMin) / uSpan;
            double     v   = (glm::dot(rel, axisV) - vMin) / vSpan;
            uParams[pIdx]  = u;
            vParams[pIdx]  = v;

            std::vector<double> bu = evaluateBasis(knotsU, numRows, degree, u);
            std::vector<double> bv = evaluateBasis(knotsV, numCols, degree, v);

            std::vector<double> weights(unknownCount, 0.0);
            for (size_t r = 0; r < numRows; r++)
            {
                if (bu[r] == 0.0)
                {
                    continue;
                }
                for (size_t c = 0; c < numCols; c++)
                {
                    weights[r * numCols + c] = bu[r] * bv[c];
                }
            }

            for (size_t a = 0; a < unknownCount; a++)
            {
                if (weights[a] == 0.0)
                {
                    continue;
                }
                rhs[a] += weights[a] * points[pIdx];
                for (size_t b = 0; b < unknownCount; b++)
                {
                    if (weights[b] == 0.0)
                    {
                        continue;
                    }
                    normalMatrix[a * unknownCount + b] += weights[a] * weights[b];
                }
            }
        }

        for (size_t r = 1; r + 1 < numRows; r++)
        {
            for (size_t c = 0; c < numCols; c++)
            {
                size_t prevIdx = (r - 1) * numCols + c;
                size_t currIdx = r * numCols + c;
                size_t nextIdx = (r + 1) * numCols + c;
                size_t idxs[3] = { prevIdx, currIdx, nextIdx };
                double vals[3] = { 1.0, -2.0, 1.0 };
                for (size_t a = 0; a < 3; a++)
                {
                    for (size_t b = 0; b < 3; b++)
                    {
                        normalMatrix[idxs[a] * unknownCount + idxs[b]] += smoothingWeight * vals[a] * vals[b];
                    }
                }
            }
        }

        for (size_t r = 0; r < numRows; r++)
        {
            for (size_t c = 1; c + 1 < numCols; c++)
            {
                size_t prevIdx = r * numCols + (c - 1);
                size_t currIdx = r * numCols + c;
                size_t nextIdx = r * numCols + (c + 1);
                size_t idxs[3] = { prevIdx, currIdx, nextIdx };
                double vals[3] = { 1.0, -2.0, 1.0 };
                for (size_t a = 0; a < 3; a++)
                {
                    for (size_t b = 0; b < 3; b++)
                    {
                        normalMatrix[idxs[a] * unknownCount + idxs[b]] += smoothingWeight * vals[a] * vals[b];
                    }
                }
            }
        }

        for (size_t i = 0; i < unknownCount; i++)
        {
            normalMatrix[i * unknownCount + i] += 1e-8;
        }

        if (!solveLinearSystem(normalMatrix, rhs, unknownCount))
        {
            return result;
        }

        double maxDev = 0.0;
        for (size_t pIdx = 0; pIdx < points.size(); pIdx++)
        {
            std::vector<double> bu = evaluateBasis(knotsU, numRows, degree, uParams[pIdx]);
            std::vector<double> bv = evaluateBasis(knotsV, numCols, degree, vParams[pIdx]);

            glm::dvec3 evalPt(0.0);
            for (size_t r = 0; r < numRows; r++)
            {
                for (size_t c = 0; c < numCols; c++)
                {
                    evalPt += (bu[r] * bv[c]) * rhs[r * numCols + c];
                }
            }
            maxDev = std::max(maxDev, glm::length(evalPt - points[pIdx]));
        }

        result.valid         = true;
        result.numRows       = numRows;
        result.numCols       = numCols;
        result.controlPoints = std::move(rhs);
        result.maxDeviation  = maxDev;
        return result;
    }

    bool NurbsSurfaceFit::isValid() const
    {
        return valid;
    }

    size_t NurbsSurfaceFit::getNumRows() const
    {
        return numRows;
    }

    size_t NurbsSurfaceFit::getNumCols() const
    {
        return numCols;
    }

    const std::vector<glm::dvec3>& NurbsSurfaceFit::getControlPoints() const
    {
        return controlPoints;
    }

    double NurbsSurfaceFit::getMaxDeviation() const
    {
        return maxDeviation;
    }
}

#ifdef ISTESTPROJECT
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

TEST_CASE("NurbsSurfaceFit/CurvedPatch")
{
    CADacomb::Triangulation mesh;
    const size_t            gridN = 9;
    for (size_t y = 0; y < gridN; y++)
    {
        for (size_t x = 0; x < gridN; x++)
        {
            double px = static_cast<double>(x) / static_cast<double>(gridN - 1);
            double py = static_cast<double>(y) / static_cast<double>(gridN - 1);
            double pz = 0.2 * std::sin(px * 3.14159) + 0.1 * py * py;
            mesh.vertices.push_back(glm::dvec3(px, py, pz));
        }
    }

    std::vector<size_t> cluster;
    for (size_t y = 0; y + 1 < gridN; y++)
    {
        for (size_t x = 0; x + 1 < gridN; x++)
        {
            size_t i0 = y * gridN + x;
            size_t i1 = i0 + 1;
            size_t i2 = i0 + gridN;
            size_t i3 = i2 + 1;
            cluster.push_back(mesh.indices.size() / 3);
            mesh.indices.push_back(i0);
            mesh.indices.push_back(i1);
            mesh.indices.push_back(i3);
            cluster.push_back(mesh.indices.size() / 3);
            mesh.indices.push_back(i0);
            mesh.indices.push_back(i3);
            mesh.indices.push_back(i2);
        }
    }

    CADacomb::CAD::NurbsSurfaceFit fit = CADacomb::CAD::NurbsSurfaceFit::fit(mesh, cluster, 6, 6, 1e-4);
    REQUIRE(fit.isValid());
    REQUIRE(fit.getNumRows() == 6);
    REQUIRE(fit.getNumCols() == 6);
    REQUIRE(fit.getControlPoints().size() == 36);
    REQUIRE(fit.getMaxDeviation() < 1e-2);
}
#endif

#include "CADacomb/Triangle/MeshSimplifier.h"

#include "CADacomb/Triangle/EdgeCollapseCandidate.h"
#include "CADacomb/Triangle/SimplificationTriangle.h"
#include "CADacomb/Util/ProgressBar.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <glm/gtc/matrix_inverse.hpp>
#include <limits>
#include <map>
#include <queue>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

namespace CADacomb
{
    MeshSimplifier::MeshSimplifier() {}

    MeshSimplifier::~MeshSimplifier() {}

    glm::dmat4 MeshSimplifier::computePlaneQuadric(const glm::dvec3& p0, const glm::dvec3& p1, const glm::dvec3& p2)
    {
        glm::dvec3 normal = glm::cross(p1 - p0, p2 - p0);
        double     len    = glm::length(normal);
        if (len <= 1e-15)
        {
            return glm::dmat4(0.0);
        }
        normal /= len;
        double     d = -glm::dot(normal, p0);
        glm::dvec4 plane(normal.x, normal.y, normal.z, d);

        glm::dmat4 q(0.0);
        for (int r = 0; r < 4; r++)
        {
            for (int c = 0; c < 4; c++)
            {
                q[c][r] = plane[r] * plane[c];
            }
        }
        return q;
    }

    double MeshSimplifier::evaluateQuadric(const glm::dmat4& q, const glm::dvec3& p)
    {
        glm::dvec4 v(p.x, p.y, p.z, 1.0);
        double     err = glm::dot(v, q * v);
        return err < 0.0 ? 0.0 : err;
    }

    bool MeshSimplifier::computeOptimalPosition(const glm::dmat4& q, const glm::dvec3& pA, const glm::dvec3& pB, glm::dvec3& outPos, double& outError)
    {
        glm::dmat3 upper(q[0][0], q[0][1], q[0][2], q[1][0], q[1][1], q[1][2], q[2][0], q[2][1], q[2][2]);
        double     det = glm::determinant(upper);

        if (std::abs(det) > 1e-10)
        {
            glm::dvec3 rhs(-q[3][0], -q[3][1], -q[3][2]);
            glm::dvec3 solved = glm::inverse(upper) * rhs;
            if (std::isfinite(solved.x) && std::isfinite(solved.y) && std::isfinite(solved.z))
            {
                double     edgeLen = glm::length(pB - pA);
                glm::dvec3 mid     = 0.5 * (pA + pB);
                if (glm::length(solved - mid) <= std::max(edgeLen * 2.0, 1e-6))
                {
                    outPos   = solved;
                    outError = evaluateQuadric(q, outPos);
                    return true;
                }
            }
        }

        glm::dvec3 mid  = 0.5 * (pA + pB);
        double     errA = evaluateQuadric(q, pA);
        double     errB = evaluateQuadric(q, pB);
        double     errM = evaluateQuadric(q, mid);

        if (errM <= errA && errM <= errB)
        {
            outPos   = mid;
            outError = errM;
        }
        else if (errA <= errB)
        {
            outPos   = pA;
            outError = errA;
        }
        else
        {
            outPos   = pB;
            outError = errB;
        }
        return true;
    }

    void MeshSimplifier::weldVertices(const Triangulation& input, std::vector<glm::dvec3>& outVertices, std::vector<size_t>& outIndices)
    {
        outVertices.clear();
        outIndices.clear();
        if (input.vertices.empty() || input.indices.size() < 3)
        {
            return;
        }

        auto   aabb = input.getAABB();
        double diag = glm::length(aabb.second);
        double eps  = std::max(diag * 1e-9, 1e-12);
        double inv  = 1.0 / eps;

        std::map<std::tuple<int64_t, int64_t, int64_t>, size_t> quantMap;
        std::vector<size_t>                                     remap(input.vertices.size(), 0);

        for (size_t i = 0; i < input.vertices.size(); i++)
        {
            const glm::dvec3&                     v = input.vertices[i];
            std::tuple<int64_t, int64_t, int64_t> key(static_cast<int64_t>(std::llround(v.x * inv)), static_cast<int64_t>(std::llround(v.y * inv)), static_cast<int64_t>(std::llround(v.z * inv)));
            auto                                  it = quantMap.find(key);
            if (it == quantMap.end())
            {
                size_t newIdx = outVertices.size();
                quantMap[key] = newIdx;
                outVertices.push_back(v);
                remap[i] = newIdx;
            }
            else
            {
                remap[i] = it->second;
            }
        }

        outIndices.reserve(input.indices.size());
        for (size_t i = 0; i + 2 < input.indices.size(); i += 3)
        {
            size_t a = remap[input.indices[i + 0]];
            size_t b = remap[input.indices[i + 1]];
            size_t c = remap[input.indices[i + 2]];
            if (a != b && b != c && c != a)
            {
                outIndices.push_back(a);
                outIndices.push_back(b);
                outIndices.push_back(c);
            }
        }
    }

    std::unique_ptr<Triangulation> MeshSimplifier::simplifyByPercent(const Triangulation& input, double targetPercent)
    {
        double clamped       = std::clamp(targetPercent, 0.0, 100.0);
        size_t inputTriCount = input.indices.size() / 3;
        size_t targetCount   = static_cast<size_t>(std::llround(static_cast<double>(inputTriCount) * (clamped / 100.0)));
        if (inputTriCount > 0 && clamped > 0.0 && targetCount < 4)
        {
            targetCount = std::min<size_t>(4, inputTriCount);
        }
        return simplifyByTriangleCount(input, targetCount);
    }

    std::unique_ptr<Triangulation> MeshSimplifier::simplifyByTriangleCount(const Triangulation& input, size_t targetTriangleCount)
    {
        auto result = std::make_unique<Triangulation>();
        if (input.vertices.empty() || input.indices.size() < 3)
        {
            return result;
        }

        std::vector<glm::dvec3> verts;
        std::vector<size_t>     indices;
        weldVertices(input, verts, indices);

        size_t initialTriCount = indices.size() / 3;
        if (initialTriCount == 0)
        {
            return result;
        }

        if (targetTriangleCount >= initialTriCount)
        {
            result->vertices = std::move(verts);
            result->indices  = std::move(indices);
            return result;
        }

        size_t targetTris = std::max<size_t>(targetTriangleCount, 1);

        size_t                  vCount = verts.size();
        std::vector<glm::dmat4> quadrics(vCount, glm::dmat4(0.0));
        std::vector<bool>       vertDeleted(vCount, false);
        std::vector<uint32_t>   vertVersion(vCount, 0);

        std::vector<SimplificationTriangle> tris(initialTriCount);
        std::vector<std::vector<size_t>>    vertTris(vCount);

        for (size_t t = 0; t < initialTriCount; t++)
        {
            size_t a     = indices[t * 3 + 0];
            size_t b     = indices[t * 3 + 1];
            size_t c     = indices[t * 3 + 2];
            tris[t].v[0] = a;
            tris[t].v[1] = b;
            tris[t].v[2] = c;
            vertTris[a].push_back(t);
            vertTris[b].push_back(t);
            vertTris[c].push_back(t);

            glm::dmat4 q = computePlaneQuadric(verts[a], verts[b], verts[c]);
            quadrics[a] += q;
            quadrics[b] += q;
            quadrics[c] += q;
        }

        auto makeEdgeKey = [](size_t a, size_t b) -> uint64_t
        {
            if (a > b)
            {
                std::swap(a, b);
            }
            return (static_cast<uint64_t>(a) << 32) | static_cast<uint64_t>(b);
        };

        std::unordered_map<uint64_t, int> edgeFaceCount;
        edgeFaceCount.reserve(initialTriCount * 2);
        for (size_t t = 0; t < initialTriCount; t++)
        {
            for (int k = 0; k < 3; k++)
            {
                edgeFaceCount[makeEdgeKey(tris[t].v[k], tris[t].v[(k + 1) % 3])]++;
            }
        }

        for (size_t t = 0; t < initialTriCount; t++)
        {
            for (int k = 0; k < 3; k++)
            {
                size_t a = tris[t].v[k];
                size_t b = tris[t].v[(k + 1) % 3];
                if (edgeFaceCount[makeEdgeKey(a, b)] == 1)
                {
                    glm::dvec3 faceNormal = glm::cross(verts[tris[t].v[1]] - verts[tris[t].v[0]], verts[tris[t].v[2]] - verts[tris[t].v[0]]);
                    glm::dvec3 edgeVec    = verts[b] - verts[a];
                    glm::dvec3 bNormal    = glm::cross(edgeVec, faceNormal);
                    double     len        = glm::length(bNormal);
                    if (len > 1e-15)
                    {
                        bNormal /= len;
                        double     d = -glm::dot(bNormal, verts[a]);
                        glm::dvec4 plane(bNormal.x, bNormal.y, bNormal.z, d);
                        glm::dmat4 bq(0.0);
                        for (int r = 0; r < 4; r++)
                        {
                            for (int c = 0; c < 4; c++)
                            {
                                bq[c][r] = plane[r] * plane[c] * 100.0;
                            }
                        }
                        quadrics[a] += bq;
                        quadrics[b] += bq;
                    }
                }
            }
        }

        std::priority_queue<EdgeCollapseCandidate, std::vector<EdgeCollapseCandidate>, std::greater<EdgeCollapseCandidate>> heap;

        auto pushEdge = [&](size_t a, size_t b)
        {
            if (a == b || vertDeleted[a] || vertDeleted[b])
            {
                return;
            }
            if (a > b)
            {
                std::swap(a, b);
            }
            glm::dmat4 qSum = quadrics[a] + quadrics[b];
            glm::dvec3 pos;
            double     err = 0.0;
            computeOptimalPosition(qSum, verts[a], verts[b], pos, err);
            heap.push(EdgeCollapseCandidate(err, a, b, vertVersion[a], vertVersion[b], pos));
        };

        for (const auto& kv : edgeFaceCount)
        {
            size_t a = static_cast<size_t>(kv.first >> 32);
            size_t b = static_cast<size_t>(kv.first & 0xFFFFFFFFu);
            pushEdge(a, b);
        }

        auto causesNormalFlip = [&](size_t movedVert, size_t otherVert, const glm::dvec3& newPos) -> bool
        {
            for (size_t tIdx : vertTris[movedVert])
            {
                const SimplificationTriangle& tri = tris[tIdx];
                if (tri.deleted)
                {
                    continue;
                }
                if (tri.v[0] == otherVert || tri.v[1] == otherVert || tri.v[2] == otherVert)
                {
                    continue;
                }
                glm::dvec3 oldP[3] = { verts[tri.v[0]], verts[tri.v[1]], verts[tri.v[2]] };
                glm::dvec3 newP[3] = { oldP[0], oldP[1], oldP[2] };
                for (int k = 0; k < 3; k++)
                {
                    if (tri.v[k] == movedVert)
                    {
                        newP[k] = newPos;
                    }
                }
                glm::dvec3 oldN = glm::cross(oldP[1] - oldP[0], oldP[2] - oldP[0]);
                glm::dvec3 newN = glm::cross(newP[1] - newP[0], newP[2] - newP[0]);
                double     oldL = glm::length(oldN);
                double     newL = glm::length(newN);
                if (oldL > 1e-15 && newL > 1e-15)
                {
                    if (glm::dot(oldN / oldL, newN / newL) < 0.1)
                    {
                        return true;
                    }
                }
            }
            return false;
        };

        size_t      activeTris = initialTriCount;
        size_t      toRemove   = initialTriCount - targetTris;
        ProgressBar bar(toRemove > 0 ? toRemove : 1, "Simplifying mesh");

        while (activeTris > targetTris && !heap.empty() && !bar.isCanceled())
        {
            EdgeCollapseCandidate top = heap.top();
            heap.pop();

            if (vertDeleted[top.vA] || vertDeleted[top.vB])
            {
                continue;
            }
            if (vertVersion[top.vA] != top.verA || vertVersion[top.vB] != top.verB)
            {
                continue;
            }

            if (causesNormalFlip(top.vA, top.vB, top.targetPos) || causesNormalFlip(top.vB, top.vA, top.targetPos))
            {
                continue;
            }

            size_t keep   = top.vA;
            size_t remove = top.vB;

            verts[keep]         = top.targetPos;
            quadrics[keep]      = quadrics[keep] + quadrics[remove];
            vertDeleted[remove] = true;
            vertVersion[keep]++;

            size_t removedThisStep = 0;
            for (size_t tIdx : vertTris[remove])
            {
                SimplificationTriangle& tri = tris[tIdx];
                if (tri.deleted)
                {
                    continue;
                }
                bool hasKeep = (tri.v[0] == keep || tri.v[1] == keep || tri.v[2] == keep);
                if (hasKeep)
                {
                    tri.deleted = true;
                    activeTris--;
                    removedThisStep++;
                }
                else
                {
                    for (int k = 0; k < 3; k++)
                    {
                        if (tri.v[k] == remove)
                        {
                            tri.v[k] = keep;
                        }
                    }
                    vertTris[keep].push_back(tIdx);
                }
            }
            vertTris[remove].clear();

            if (removedThisStep > 0)
            {
                bar.addProgress(removedThisStep);
            }

            std::vector<size_t> cleanedTris;
            cleanedTris.reserve(vertTris[keep].size());
            std::vector<size_t> neighbors;
            for (size_t tIdx : vertTris[keep])
            {
                const SimplificationTriangle& tri = tris[tIdx];
                if (tri.deleted)
                {
                    continue;
                }
                cleanedTris.push_back(tIdx);
                for (int k = 0; k < 3; k++)
                {
                    if (tri.v[k] != keep)
                    {
                        neighbors.push_back(tri.v[k]);
                    }
                }
            }
            vertTris[keep] = std::move(cleanedTris);

            std::sort(neighbors.begin(), neighbors.end());
            neighbors.erase(std::unique(neighbors.begin(), neighbors.end()), neighbors.end());
            for (size_t nb : neighbors)
            {
                pushEdge(keep, nb);
            }
        }

        std::vector<size_t> outRemap(vCount, std::numeric_limits<size_t>::max());
        result->indices.reserve(activeTris * 3);
        for (const SimplificationTriangle& tri : tris)
        {
            if (tri.deleted)
            {
                continue;
            }
            if (tri.v[0] == tri.v[1] || tri.v[1] == tri.v[2] || tri.v[2] == tri.v[0])
            {
                continue;
            }
            for (int k = 0; k < 3; k++)
            {
                size_t oldV = tri.v[k];
                if (outRemap[oldV] == std::numeric_limits<size_t>::max())
                {
                    outRemap[oldV] = result->vertices.size();
                    result->vertices.push_back(verts[oldV]);
                }
                result->indices.push_back(outRemap[oldV]);
            }
        }

        return result;
    }
}

#ifdef ISTESTPROJECT
#include "CADacomb/Triangle/BaseShapeGenerator.h"

#include <catch2/catch_test_macros.hpp>

TEST_CASE("MeshSimplifier/SimplifyByPercentAndTriangleCount")
{
    auto   sphere   = CADacomb::BaseShapeGenerator::sphere(24, 12, 1.0f);
    size_t origTris = sphere.indices.size() / 3;
    REQUIRE(origTris > 100);

    auto byPercent = CADacomb::MeshSimplifier::simplifyByPercent(sphere, 50.0);
    REQUIRE(byPercent != nullptr);
    size_t pctTris = byPercent->indices.size() / 3;
    CHECK(pctTris > 0);
    CHECK(pctTris <= origTris / 2 + 2);
    CHECK(byPercent->indices.size() % 3 == 0);

    auto byCount = CADacomb::MeshSimplifier::simplifyByTriangleCount(sphere, 80);
    REQUIRE(byCount != nullptr);
    size_t countTris = byCount->indices.size() / 3;
    CHECK(countTris > 0);
    CHECK(countTris <= 80);
    CHECK(byCount->indices.size() % 3 == 0);

    for (size_t idx : byCount->indices)
    {
        CHECK(idx < byCount->vertices.size());
    }
}
#endif

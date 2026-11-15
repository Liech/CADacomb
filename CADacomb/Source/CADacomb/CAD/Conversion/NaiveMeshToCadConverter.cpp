#include "NaiveMeshToCadConverter.h"

#include "CADacomb/CAD/Interface/Kernel.h"
#include "CADacomb/CAD/Interface/Shape.h"
#include "CADacomb/Triangle/Triangulation.h"

#include <glm/glm.hpp>
#include <stdexcept>
#include <vector>

namespace CADacomb::CAD
{
    NaiveMeshToCadConverter::NaiveMeshToCadConverter() {}

    NaiveMeshToCadConverter::~NaiveMeshToCadConverter() {}

    std::unique_ptr<Shape> NaiveMeshToCadConverter::convert(const Kernel& kernel, const Triangulation& mesh, double stitchTolerance, bool tryMakeSolid) const
    {
        if (mesh.indices.size() < 3 || mesh.vertices.empty())
        {
            throw std::runtime_error("NaiveMeshToCadConverter::convert: triangulation is empty");
        }

        size_t                              triangleCount = mesh.indices.size() / 3;
        std::vector<std::unique_ptr<Shape>> faceShapes;
        faceShapes.reserve(triangleCount);

        for (size_t i = 0; i < triangleCount; i++)
        {
            const glm::dvec3& p0 = mesh.vertices[mesh.indices[i * 3 + 0]];
            const glm::dvec3& p1 = mesh.vertices[mesh.indices[i * 3 + 1]];
            const glm::dvec3& p2 = mesh.vertices[mesh.indices[i * 3 + 2]];

            glm::dvec3 crossVec = glm::cross(p1 - p0, p2 - p0);
            double     len      = glm::length(crossVec);
            if (len < 1e-12)
            {
                continue;
            }

            glm::dvec3                           normal = crossVec / len;
            std::vector<std::vector<glm::dvec3>> loops  = { { p0, p1, p2 } };
            faceShapes.push_back(kernel.createPlanarFace(p0, normal, loops));
        }

        if (faceShapes.empty())
        {
            throw std::runtime_error("NaiveMeshToCadConverter::convert: no valid triangles to convert");
        }

        std::vector<const Shape*> facePtrs;
        facePtrs.reserve(faceShapes.size());
        for (size_t i = 0; i < faceShapes.size(); i++)
        {
            facePtrs.push_back(faceShapes[i].get());
        }

        std::unique_ptr<Shape> merged   = kernel.mergeShapes(facePtrs);
        std::unique_ptr<Shape> stitched = kernel.stitch(*merged, stitchTolerance, tryMakeSolid);
        kernel.unifyNormals(*stitched);
        return stitched;
    }
}

#ifdef ISTESTPROJECT
#include "CADacomb/CAD/Sm/KernelSm.h"
#include "CADacomb/Triangle/BaseShapeGenerator.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

TEST_CASE("NaiveMeshToCadConverter/CubeToCad")
{
    CADacomb::Triangulation                 boxTri = CADacomb::BaseShapeGenerator::cube();
    CADacomb::CAD::KernelSm                 kernel;
    CADacomb::CAD::NaiveMeshToCadConverter  converter;
    std::unique_ptr<CADacomb::CAD::Shape>   shape = converter.convert(kernel, boxTri, 1e-4, true);

    REQUIRE(shape != nullptr);

    CADacomb::AABB3 triBounds = boxTri.getAABB();
    CADacomb::AABB3 bounds    = shape->getAABB();
    REQUIRE(bounds.min.x == Catch::Approx(triBounds.min.x).margin(1e-3));
    REQUIRE(bounds.min.y == Catch::Approx(triBounds.min.y).margin(1e-3));
    REQUIRE(bounds.min.z == Catch::Approx(triBounds.min.z).margin(1e-3));
    REQUIRE(bounds.max.x == Catch::Approx(triBounds.max.x).margin(1e-3));
    REQUIRE(bounds.max.y == Catch::Approx(triBounds.max.y).margin(1e-3));
    REQUIRE(bounds.max.z == Catch::Approx(triBounds.max.z).margin(1e-3));
}
#endif

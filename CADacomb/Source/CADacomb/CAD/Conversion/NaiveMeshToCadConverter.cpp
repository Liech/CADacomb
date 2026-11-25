#include "NaiveMeshToCadConverter.h"

#include "CADacomb/CAD/Interface/Kernel.h"
#include "CADacomb/CAD/Interface/Shape.h"
#include "CADacomb/Triangle/Triangulation.h"
#include "CADacomb/Util/ProgressBar.h"

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

        ProgressBar                         mainBar(4, "Converting triangles to CAD");
        size_t                              triangleCount = mesh.indices.size() / 3;
        std::vector<std::unique_ptr<Shape>> faceShapes;
        faceShapes.reserve(triangleCount);

        {
            ProgressBar faceBar(triangleCount, "Creating triangle CAD faces");
            for (size_t i = 0; i < triangleCount; i++)
            {
                if (faceBar.isCanceled())
                {
                    throw std::runtime_error("NaiveMeshToCadConverter::convert: canceled");
                }
                faceBar.addProgress(1);

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
        }
        mainBar.addProgress(1);

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

        std::unique_ptr<Shape> merged = kernel.mergeShapes(facePtrs);
        mainBar.addProgress(1);
        std::unique_ptr<Shape> stitched = kernel.stitch(*merged, stitchTolerance, tryMakeSolid);
        mainBar.addProgress(1);
        kernel.unifyNormals(*stitched);
        mainBar.addProgress(1);
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

    std::pair<glm::dvec3, glm::dvec3> triPair = boxTri.getAABB();
    CADacomb::AABB3                   triBounds(triPair.first, triPair.first + triPair.second);
    CADacomb::AABB3 bounds    = shape->getAABB();
    REQUIRE(bounds.getMin().x == Catch::Approx(triBounds.getMin().x).margin(1e-3));
    REQUIRE(bounds.getMin().y == Catch::Approx(triBounds.getMin().y).margin(1e-3));
    REQUIRE(bounds.getMin().z == Catch::Approx(triBounds.getMin().z).margin(1e-3));
    REQUIRE(bounds.getMax().x == Catch::Approx(triBounds.getMax().x).margin(1e-3));
    REQUIRE(bounds.getMax().y == Catch::Approx(triBounds.getMax().y).margin(1e-3));
    REQUIRE(bounds.getMax().z == Catch::Approx(triBounds.getMax().z).margin(1e-3));
}
#endif

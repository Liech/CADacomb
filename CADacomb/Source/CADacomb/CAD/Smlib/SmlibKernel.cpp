#include "SmlibKernel.h"

#include "SmPyError.h"
#include "SmlibSolid.h"
#include "ext_shared.h"
#include "tessellate.h"
#include <SmApiBrep.h>
#include <SmApiPrimitives.h>
#include <SmApiQueries.h>
#include <SmPoly.h>
#include <unordered_map>

namespace CADacomb::CAD
{
    SmlibKernel::SmlibKernel()
    {
        install_sm_error_callback();
    }

    SmlibKernel::~SmlibKernel() {}

    std::unique_ptr<Solid> SmlibKernel::createBox(const glm::dvec3& minCorner, const glm::dvec3& dimensions) const
    {
        SmBrep*  boxBrep = nullptr;
        SmStatus status  = SmApiCreateBox(SmVector3d(minCorner.x, minCorner.y, minCorner.z), dimensions.x, dimensions.y, dimensions.z, boxBrep);
        if (status != SM_SUCCESS)
        {
            SmPyRaise(status, "SmApiCreateBox", __FILE__, __LINE__);
        }
        return std::make_unique<SmlibSolid>(boxBrep);
    }

    std::unique_ptr<Solid> SmlibKernel::createCylinder(const glm::dvec3& baseCenter, double radius, double height) const
    {
        SmBrep*  cylinderBrep = nullptr;
        SmStatus status       = SmApiCreateCylinder(SmVector3d(baseCenter.x, baseCenter.y, baseCenter.z), radius, height, cylinderBrep);
        if (status != SM_SUCCESS)
        {
            SmPyRaise(status, "SmApiCreateCylinder", __FILE__, __LINE__);
        }
        return std::make_unique<SmlibSolid>(cylinderBrep);
    }

    std::unique_ptr<Shape> SmlibKernel::booleanUnion(const Shape& shapeA, const Shape& shapeB) const
    {
        SmBrep*  copyA      = copyBrepFromShape(shapeA);
        SmBrep*  copyB      = copyBrepFromShape(shapeB);
        SmBrep*  resultBrep = nullptr;
        SmStatus status     = SmApiBooleanUnion(copyA, copyB, resultBrep);
        if (status != SM_SUCCESS)
        {
            smlibpy::delete_brep(copyA);
            smlibpy::delete_brep(copyB);
            SmPyRaise(status, "SmApiBooleanUnion", __FILE__, __LINE__);
        }
        return std::make_unique<SmlibSolid>(resultBrep);
    }

    std::unique_ptr<Shape> SmlibKernel::booleanDifference(const Shape& shapeA, const Shape& shapeB) const
    {
        SmBrep*  copyA      = copyBrepFromShape(shapeA);
        SmBrep*  copyB      = copyBrepFromShape(shapeB);
        SmBrep*  resultBrep = nullptr;
        SmStatus status     = SmApiBooleanDifference(copyA, copyB, resultBrep);
        if (status != SM_SUCCESS)
        {
            smlibpy::delete_brep(copyA);
            smlibpy::delete_brep(copyB);
            SmPyRaise(status, "SmApiBooleanDifference", __FILE__, __LINE__);
        }
        return std::make_unique<SmlibSolid>(resultBrep);
    }

    std::unique_ptr<Shape> SmlibKernel::booleanIntersection(const Shape& shapeA, const Shape& shapeB) const
    {
        SmBrep*  copyA      = copyBrepFromShape(shapeA);
        SmBrep*  copyB      = copyBrepFromShape(shapeB);
        SmBrep*  resultBrep = nullptr;
        SmStatus status     = SmApiBooleanIntersection(copyA, copyB, resultBrep);
        if (status != SM_SUCCESS)
        {
            smlibpy::delete_brep(copyA);
            smlibpy::delete_brep(copyB);
            SmPyRaise(status, "SmApiBooleanIntersection", __FILE__, __LINE__);
        }
        return std::make_unique<SmlibSolid>(resultBrep);
    }

    std::unique_ptr<Triangulation> SmlibKernel::tessellate(const Shape& shape, const TessellationOptions& options) const
    {
        const SmlibSolid&          solid = dynamic_cast<const SmlibSolid&>(shape);
        smlibpy::TessellateOptions smOptions;
        smOptions.chord_height_tolerance      = options.getChordHeightTolerance();
        smOptions.surface_angle_tolerance_deg = options.getSurfaceAngleToleranceDeg();

        smlibpy::Tessellation tessellation = smlibpy::tessellate(solid.getBrep(), smOptions);
        if (tessellation.status != SM_SUCCESS)
        {
            SmPyRaise(tessellation.status, "smlibpy::tessellate", __FILE__, __LINE__);
        }

        std::unique_ptr<Triangulation> result = std::make_unique<Triangulation>();

        SmTArray<SmPolyVertex*> polyVertices;
        SmTArray<SmPolyFace*>   polyFaces;
        tessellation.mesh->GetPolyVertices(polyVertices);
        tessellation.mesh->GetPolyFaces(polyFaces);

        std::unordered_map<SmPolyVertex*, size_t> vertexIndexMap;
        vertexIndexMap.reserve(polyVertices.GetSize());
        result->vertices.reserve(polyVertices.GetSize());

        for (ULONG vertexIndex = 0; vertexIndex < polyVertices.GetSize(); vertexIndex++)
        {
            SmPolyVertex*    polyVertex = polyVertices[vertexIndex];
            const SmPoint3d& point      = polyVertex->GetPoint();
            vertexIndexMap[polyVertex]  = vertexIndex;
            result->vertices.push_back(glm::dvec3(point.x, point.y, point.z));
        }

        for (ULONG faceIndex = 0; faceIndex < polyFaces.GetSize(); faceIndex++)
        {
            SmPolyLoop* outerLoop = polyFaces[faceIndex]->GetOuterPolyLoop();
            if (outerLoop == nullptr)
            {
                continue;
            }
            SmTArray<SmPolyEdge*> loopEdges;
            outerLoop->GetPolyEdges(loopEdges);
            if (loopEdges.GetSize() < 3)
            {
                continue;
            }

            size_t firstVertexIndex = vertexIndexMap[loopEdges[0]->GetStartPolyVertex()];
            for (ULONG edgeIndex = 1; edgeIndex + 1 < loopEdges.GetSize(); edgeIndex++)
            {
                size_t secondVertexIndex = vertexIndexMap[loopEdges[edgeIndex]->GetStartPolyVertex()];
                size_t thirdVertexIndex  = vertexIndexMap[loopEdges[edgeIndex + 1]->GetStartPolyVertex()];
                result->indices.push_back(firstVertexIndex);
                result->indices.push_back(secondVertexIndex);
                result->indices.push_back(thirdVertexIndex);
            }
        }

        smlibpy::delete_polybrep(tessellation.mesh);
        return result;
    }

    std::vector<std::unique_ptr<Shape>> SmlibKernel::loadStepFile(const std::string& filePath) const
    {
        return std::vector<std::unique_ptr<Shape>>();
    }

    void SmlibKernel::saveStepFile(const Shape& shape, const std::string& filePath, const std::string& productName) const {}

    SmBrep* SmlibKernel::copyBrepFromShape(const Shape& shape) const
    {
        const SmlibSolid& solid  = dynamic_cast<const SmlibSolid&>(shape);
        SmBrep*           copied = nullptr;
        SmStatus          status = SmApiBrepCopy(solid.getBrep(), copied);
        if (status != SM_SUCCESS)
        {
            SmPyRaise(status, "SmApiBrepCopy", __FILE__, __LINE__);
        }
        return copied;
    }
}

#ifdef ISTESTPROJECT
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

TEST_CASE("SmlibKernel/CreateBoxWithHoleAndTessellate")
{
    CADacomb::CAD::SmlibKernel kernel;

    std::unique_ptr<CADacomb::CAD::Solid> box      = kernel.createBox(glm::dvec3(-30.0, -20.0, -10.0), glm::dvec3(60.0, 40.0, 20.0));
    std::unique_ptr<CADacomb::CAD::Solid> cylinder = kernel.createCylinder(glm::dvec3(0.0, 0.0, -15.0), 10.0, 30.0);
    std::unique_ptr<CADacomb::CAD::Shape> result   = kernel.booleanDifference(*box, *cylinder);

    REQUIRE(result != nullptr);
    REQUIRE(result->getShapeType() == CADacomb::CAD::ShapeType::Solid);

    const CADacomb::CAD::Solid& resultSolid = dynamic_cast<const CADacomb::CAD::Solid&>(*result);
    REQUIRE(resultSolid.isManifold());

    CADacomb::AABB3 bounds = resultSolid.getAABB();
    REQUIRE(bounds.getMin().x == Catch::Approx(-30.0));
    REQUIRE(bounds.getMin().y == Catch::Approx(-20.0));
    REQUIRE(bounds.getMin().z == Catch::Approx(-10.0));
    REQUIRE(bounds.getMax().x == Catch::Approx(30.0));
    REQUIRE(bounds.getMax().y == Catch::Approx(20.0));
    REQUIRE(bounds.getMax().z == Catch::Approx(10.0));

    double expectedVolume = (60.0 * 40.0 * 20.0) - (3.14159265358979323846 * 10.0 * 10.0 * 20.0);
    REQUIRE(resultSolid.getVolume() == Catch::Approx(expectedVolume).epsilon(1e-3));
    REQUIRE(resultSolid.getSurfaceArea() > 0.0);

    CADacomb::CAD::TessellationOptions options;
    options.setChordHeightTolerance(0.05);
    options.setSurfaceAngleToleranceDeg(15.0);

    std::unique_ptr<CADacomb::Triangulation> mesh = kernel.tessellate(*result, options);
    REQUIRE(mesh != nullptr);
    REQUIRE(!mesh->vertices.empty());
    REQUIRE(!mesh->indices.empty());
    REQUIRE(mesh->indices.size() % 3 == 0);
}
#endif

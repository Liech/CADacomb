#include "KernelSm.h"

#include "SmPyError.h"
#include "SolidSm.h"
#include "ext_shared.h"
#include "tessellate.h"
#include <SmApiBrep.h>
#include <SmApiPrimitives.h>
#include <SmApiQueries.h>
#include <SmPoly.h>
#include <unordered_map>

namespace CADacomb::CAD
{
    KernelSm::KernelSm()
    {
        install_sm_error_callback();
    }

    KernelSm::~KernelSm() {}

    std::unique_ptr<Solid> KernelSm::createBox(const glm::dvec3& minCorner, const glm::dvec3& dimensions) const
    {
        SmBrep*  boxBrep = nullptr;
        SmStatus status  = SmApiCreateBox(SmVector3d(minCorner.x, minCorner.y, minCorner.z), dimensions.x, dimensions.y, dimensions.z, boxBrep);
        if (status != SM_SUCCESS)
        {
            SmPyRaise(status, "SmApiCreateBox", __FILE__, __LINE__);
        }
        return std::make_unique<SolidSm>(boxBrep);
    }

    std::unique_ptr<Solid> KernelSm::createCylinder(const glm::dvec3& baseCenter, double radius, double height) const
    {
        SmBrep*  cylinderBrep = nullptr;
        SmStatus status       = SmApiCreateCylinder(SmVector3d(baseCenter.x, baseCenter.y, baseCenter.z), radius, height, cylinderBrep);
        if (status != SM_SUCCESS)
        {
            SmPyRaise(status, "SmApiCreateCylinder", __FILE__, __LINE__);
        }
        return std::make_unique<SolidSm>(cylinderBrep);
    }

    std::unique_ptr<Shape> KernelSm::booleanUnion(const Shape& shapeA, const Shape& shapeB) const
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
        return std::make_unique<SolidSm>(resultBrep);
    }

    std::unique_ptr<Shape> KernelSm::booleanDifference(const Shape& shapeA, const Shape& shapeB) const
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
        return std::make_unique<SolidSm>(resultBrep);
    }

    std::unique_ptr<Shape> KernelSm::booleanIntersection(const Shape& shapeA, const Shape& shapeB) const
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
        return std::make_unique<SolidSm>(resultBrep);
    }

    std::unique_ptr<Triangulation> KernelSm::tessellate(const Shape& shape, const TessellationOptions& options) const
    {
        const SolidSm&             solid = dynamic_cast<const SolidSm&>(shape);
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

    std::vector<std::unique_ptr<Shape>> KernelSm::loadStepFile(const std::string& filePath) const
    {
        return std::vector<std::unique_ptr<Shape>>();
    }

    void KernelSm::saveStepFile(const Shape& shape, const std::string& filePath, const std::string& productName) const {}

    SmBrep* KernelSm::copyBrepFromShape(const Shape& shape) const
    {
        const SolidSm& solid  = dynamic_cast<const SolidSm&>(shape);
        SmBrep*        copied = nullptr;
        SmStatus       status = SmApiBrepCopy(solid.getBrep(), copied);
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

TEST_CASE("KernelSm/CreateBoxWithHoleAndTessellate")
{
    CADacomb::CAD::KernelSm kernel;

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

TEST_CASE("KernelSm/SubshapeHierarchyTraversal")
{
    CADacomb::CAD::KernelSm kernel;

    std::unique_ptr<CADacomb::CAD::Solid> box      = kernel.createBox(glm::dvec3(-30.0, -20.0, -10.0), glm::dvec3(60.0, 40.0, 20.0));
    std::unique_ptr<CADacomb::CAD::Solid> cylinder = kernel.createCylinder(glm::dvec3(0.0, 0.0, -15.0), 10.0, 30.0);
    std::unique_ptr<CADacomb::CAD::Shape> result   = kernel.booleanDifference(*box, *cylinder);

    const CADacomb::CAD::Solid&                        resultSolid = dynamic_cast<const CADacomb::CAD::Solid&>(*result);
    std::vector<std::unique_ptr<CADacomb::CAD::Shell>> shells      = resultSolid.getShells();
    REQUIRE(shells.size() == 1);
    REQUIRE(shells[0]->getShapeType() == CADacomb::CAD::ShapeType::Shell);
    REQUIRE(shells[0]->isClosed());

    std::vector<std::unique_ptr<CADacomb::CAD::Face>> faces = shells[0]->getFaces();
    REQUIRE(faces.size() >= 7);

    double totalFaceArea  = 0.0;
    size_t planeCount     = 0;
    size_t cylinderCount  = 0;
    size_t innerWireCount = 0;

    for (size_t faceIndex = 0; faceIndex < faces.size(); faceIndex++)
    {
        const CADacomb::CAD::Face& face = *faces[faceIndex];
        REQUIRE(face.getShapeType() == CADacomb::CAD::ShapeType::Face);
        totalFaceArea += face.getSurfaceArea();

        if (face.getSurfaceType() == CADacomb::CAD::SurfaceType::Plane)
        {
            planeCount++;
        }
        else if (face.getSurfaceType() == CADacomb::CAD::SurfaceType::Cylinder)
        {
            cylinderCount++;
        }

        std::vector<std::unique_ptr<CADacomb::CAD::Wire>> wires = face.getWires();
        REQUIRE(!wires.empty());
        REQUIRE(wires[0]->getShapeType() == CADacomb::CAD::ShapeType::Wire);
        REQUIRE(wires[0]->isOuterWire());
        REQUIRE(wires[0]->isClosed());

        for (size_t wireIndex = 0; wireIndex < wires.size(); wireIndex++)
        {
            if (!wires[wireIndex]->isOuterWire())
            {
                innerWireCount++;
            }

            std::vector<std::unique_ptr<CADacomb::CAD::Edge>> edges = wires[wireIndex]->getEdges();
            REQUIRE(!edges.empty());

            for (size_t edgeIndex = 0; edgeIndex < edges.size(); edgeIndex++)
            {
                const CADacomb::CAD::Edge& edge = *edges[edgeIndex];
                REQUIRE(edge.getShapeType() == CADacomb::CAD::ShapeType::Edge);
                REQUIRE((edge.getCurveType() == CADacomb::CAD::CurveType::Line || edge.getCurveType() == CADacomb::CAD::CurveType::Circle));

                std::unique_ptr<CADacomb::CAD::Vertex> startVertex = edge.getStart();
                std::unique_ptr<CADacomb::CAD::Vertex> endVertex   = edge.getEnd();
                REQUIRE(startVertex != nullptr);
                REQUIRE(endVertex != nullptr);
                REQUIRE(startVertex->getShapeType() == CADacomb::CAD::ShapeType::Vertex);
                REQUIRE(endVertex->getShapeType() == CADacomb::CAD::ShapeType::Vertex);
            }
        }
    }

    REQUIRE(planeCount == 6);
    REQUIRE(cylinderCount >= 1);
    REQUIRE(innerWireCount == 2);
    REQUIRE(totalFaceArea == Catch::Approx(resultSolid.getSurfaceArea()).epsilon(1e-3));
}
#endif

#include "VertexSm.h"

#include "BrepTransformSm.h"
#include <SmVertex.h>

namespace CADacomb::CAD
{
    VertexSm::VertexSm(const std::shared_ptr<SmBrep>& brepInput, SmVertex* vertexInput)
      : brep(brepInput)
      , vertex(vertexInput)
    {
    }

    VertexSm::~VertexSm() {}

    ShapeType VertexSm::getShapeType() const
    {
        return ShapeType::Vertex;
    }

    AABB3 VertexSm::getAABB() const
    {
        glm::dvec3 pos = getPosition();
        return AABB3(pos, pos);
    }

    void VertexSm::transform(const glm::dmat4& transformationMatrix)
    {
        BrepTransformSm::transform(brep.get(), transformationMatrix);
    }

    void VertexSm::translate(const glm::dvec3& translationVector)
    {
        BrepTransformSm::translate(brep.get(), translationVector);
    }

    void VertexSm::rotate(const glm::dvec3& axisOrigin, const glm::dvec3& axisDirection, double angleRadians)
    {
        BrepTransformSm::rotate(brep.get(), axisOrigin, axisDirection, angleRadians);
    }

    void VertexSm::scale(const glm::dvec3& centerPoint, double scaleFactor)
    {
        BrepTransformSm::scale(brep.get(), centerPoint, scaleFactor);
    }

    glm::dvec3 VertexSm::getPosition() const
    {
        SmPoint3d point = vertex->GetPoint();
        return glm::dvec3(point.x, point.y, point.z);
    }

    std::vector<std::unique_ptr<Shape>> VertexSm::getChildren() const
    {
        return std::vector<std::unique_ptr<Shape>>();
    }

    SmVertex* VertexSm::getVertex() const
    {
        return vertex;
    }
}

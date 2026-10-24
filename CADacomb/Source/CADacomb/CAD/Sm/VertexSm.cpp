#include "VertexSm.h"

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

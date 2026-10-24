#pragma once

#include "CADacomb/CAD/Interface/Vertex.h"
#include <memory>

class SmBrep;
class SmVertex;

namespace CADacomb::CAD
{
    class VertexSm : public Vertex
    {
      public:
        VertexSm(const std::shared_ptr<SmBrep>& brepInput, SmVertex* vertexInput);
        virtual ~VertexSm();

        ShapeType getShapeType() const override;
        AABB3     getAABB() const override;

        glm::dvec3 getPosition() const override;

        std::vector<std::unique_ptr<Shape>> getChildren() const override;

        SmVertex* getVertex() const;

      private:
        std::shared_ptr<SmBrep> brep;
        SmVertex*               vertex;
    };
}

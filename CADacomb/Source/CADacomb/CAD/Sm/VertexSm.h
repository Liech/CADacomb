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

        void transform(const glm::dmat4& transformationMatrix) override;
        void translate(const glm::dvec3& translationVector) override;
        void rotate(const glm::dvec3& axisOrigin, const glm::dvec3& axisDirection, double angleRadians) override;
        void scale(const glm::dvec3& centerPoint, double scaleFactor) override;

        glm::dvec3 getPosition() const override;

        std::vector<std::unique_ptr<Shape>> getChildren() const override;

        SmVertex* getVertex() const;

      private:
        std::shared_ptr<SmBrep> brep;
        SmVertex*               vertex;
    };
}

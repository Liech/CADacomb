#include "EdgeSm.h"

#include "BrepTransformSm.h"
#include "SmPyError.h"
#include "VertexSm.h"
#include <SmBSplineCurve.h>
#include <SmCircle.h>
#include <SmEdge.h>
#include <SmEllipse.h>
#include <SmLine.h>

namespace CADacomb::CAD
{
    EdgeSm::EdgeSm(const std::shared_ptr<SmBrep>& brepInput, SmEdge* edgeInput, Orientation orientationInput)
      : brep(brepInput)
      , edge(edgeInput)
      , orientation(orientationInput)
    {
    }

    EdgeSm::~EdgeSm() {}

    ShapeType EdgeSm::getShapeType() const
    {
        return ShapeType::Edge;
    }

    AABB3 EdgeSm::getAABB() const
    {
        SmExtent3d box;
        SmStatus   status = edge->CalculateBoundingBox(&box, nullptr, nullptr, TRUE);
        if (status != SM_SUCCESS)
        {
            SmPyRaise(status, "SmEdge::CalculateBoundingBox", __FILE__, __LINE__);
        }
        SmPoint3d minCorner = box.GetMin();
        SmPoint3d maxCorner = box.GetMax();
        return AABB3(glm::dvec3(minCorner.x, minCorner.y, minCorner.z), glm::dvec3(maxCorner.x, maxCorner.y, maxCorner.z));
    }

    void EdgeSm::transform(const glm::dmat4& transformationMatrix)
    {
        BrepTransformSm::transform(brep.get(), transformationMatrix);
    }

    void EdgeSm::translate(const glm::dvec3& translationVector)
    {
        BrepTransformSm::translate(brep.get(), translationVector);
    }

    void EdgeSm::rotate(const glm::dvec3& axisOrigin, const glm::dvec3& axisDirection, double angleRadians)
    {
        BrepTransformSm::rotate(brep.get(), axisOrigin, axisDirection, angleRadians);
    }

    void EdgeSm::scale(const glm::dvec3& centerPoint, double scaleFactor)
    {
        BrepTransformSm::scale(brep.get(), centerPoint, scaleFactor);
    }

    CurveType EdgeSm::getCurveType() const
    {
        SmCurve* curve = edge->GetCurve();
        if (curve == nullptr)
        {
            return CurveType::Other;
        }
        switch (curve->GetType())
        {
            case SmLine_TYPE:
                return CurveType::Line;
            case SmCircle_TYPE:
                return CurveType::Circle;
            case SmEllipse_TYPE:
                return CurveType::Ellipse;
            case SmBSplineCurve_TYPE:
                return CurveType::BSpline;
            default:
                break;
        }
        if (curve->IsKindOf(SmBSplineCurve_TYPE))
        {
            return CurveType::BSpline;
        }
        return CurveType::Other;
    }

    Orientation EdgeSm::getOrientation() const
    {
        return orientation;
    }

    std::unique_ptr<Vertex> EdgeSm::getStart() const
    {
        SmVertex* startVertex = (orientation == Orientation::Reversed) ? edge->GetEndVertex() : edge->GetStartVertex();
        if (startVertex == nullptr)
        {
            return nullptr;
        }
        return std::make_unique<VertexSm>(brep, startVertex);
    }

    std::unique_ptr<Vertex> EdgeSm::getEnd() const
    {
        SmVertex* endVertex = (orientation == Orientation::Reversed) ? edge->GetStartVertex() : edge->GetEndVertex();
        if (endVertex == nullptr)
        {
            return nullptr;
        }
        return std::make_unique<VertexSm>(brep, endVertex);
    }

    std::vector<std::unique_ptr<Shape>> EdgeSm::getChildren() const
    {
        std::vector<std::unique_ptr<Shape>> children;
        std::unique_ptr<Vertex>             startVertex = getStart();
        std::unique_ptr<Vertex>             endVertex   = getEnd();
        if (startVertex != nullptr)
        {
            children.push_back(std::move(startVertex));
        }
        if (endVertex != nullptr)
        {
            children.push_back(std::move(endVertex));
        }
        return children;
    }

    double EdgeSm::getLength() const
    {
        SmCurve* curve = edge->GetCurve();
        if (curve == nullptr)
        {
            return 0.0;
        }
        double   length = 0.0;
        SmStatus status = curve->Length(edge->GetInterval(), 1e-6, length);
        if (status != SM_SUCCESS)
        {
            return curve->ApproximateLength(edge->GetInterval(), 32);
        }
        return length;
    }

    std::pair<double, double> EdgeSm::getParameterRange() const
    {
        SmExtent1d interval = edge->GetInterval();
        return { interval.GetMin(), interval.GetMax() };
    }

    glm::dvec3 EdgeSm::evaluatePoint(double parameter) const
    {
        SmCurve*  curve = edge->GetCurve();
        SmPoint3d point;
        SmStatus  status = curve->EvaluatePoint(parameter, point);
        if (status != SM_SUCCESS)
        {
            SmPyRaise(status, "SmCurve::EvaluatePoint", __FILE__, __LINE__);
        }
        return glm::dvec3(point.x, point.y, point.z);
    }

    glm::dvec3 EdgeSm::evaluateTangent(double parameter) const
    {
        SmCurve*   curve = edge->GetCurve();
        SmVector3d vectors[2];
        SmStatus   status = curve->Evaluate(parameter, 1, TRUE, vectors, TRUE);
        if (status != SM_SUCCESS)
        {
            SmPyRaise(status, "SmCurve::Evaluate", __FILE__, __LINE__);
        }
        glm::dvec3 tangent(vectors[1].x, vectors[1].y, vectors[1].z);
        if (glm::length(tangent) > 1e-12)
        {
            tangent = glm::normalize(tangent);
        }
        if (orientation == Orientation::Reversed)
        {
            tangent = -tangent;
        }
        return tangent;
    }

    SmEdge* EdgeSm::getEdge() const
    {
        return edge;
    }
}

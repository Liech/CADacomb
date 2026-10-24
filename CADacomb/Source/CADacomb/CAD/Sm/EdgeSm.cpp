#include "EdgeSm.h"

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

    SmEdge* EdgeSm::getEdge() const
    {
        return edge;
    }
}

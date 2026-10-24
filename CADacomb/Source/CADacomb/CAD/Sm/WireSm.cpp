#include "WireSm.h"

#include "EdgeSm.h"
#include "SmPyError.h"
#include <SmEdgeuse.h>
#include <SmLoopuse.h>

namespace CADacomb::CAD
{
    WireSm::WireSm(const std::shared_ptr<SmBrep>& brepInput, SmLoopuse* loopuseInput, bool outerWireInput)
      : brep(brepInput)
      , loopuse(loopuseInput)
      , outerWire(outerWireInput)
    {
    }

    WireSm::~WireSm() {}

    ShapeType WireSm::getShapeType() const
    {
        return ShapeType::Wire;
    }

    AABB3 WireSm::getAABB() const
    {
        SmExtent3d box;
        SmStatus   status = loopuse->CalculateBoundingBox(box);
        if (status != SM_SUCCESS)
        {
            SmPyRaise(status, "SmLoopuse::CalculateBoundingBox", __FILE__, __LINE__);
        }
        SmPoint3d minCorner = box.GetMin();
        SmPoint3d maxCorner = box.GetMax();
        return AABB3(glm::dvec3(minCorner.x, minCorner.y, minCorner.z), glm::dvec3(maxCorner.x, maxCorner.y, maxCorner.z));
    }

    bool WireSm::isClosed() const
    {
        return loopuse->IsClosedPtrs() != FALSE;
    }

    bool WireSm::isOuterWire() const
    {
        return outerWire;
    }

    std::vector<std::unique_ptr<Edge>> WireSm::getEdges() const
    {
        std::vector<std::unique_ptr<Edge>> edges;
        SmTArray<SmEdgeuse*>               edgeuses;
        loopuse->GetEdgeuses(edgeuses);
        edges.reserve(edgeuses.GetSize());
        for (ULONG index = 0; index < edgeuses.GetSize(); index++)
        {
            SmEdgeuse*  edgeuse     = edgeuses[index];
            Orientation orientation = (edgeuse->GetOrientation() == SM_OT_SAME) ? Orientation::Forward : Orientation::Reversed;
            edges.push_back(std::make_unique<EdgeSm>(brep, edgeuse->GetEdge(), orientation));
        }
        return edges;
    }

    std::vector<std::unique_ptr<Shape>> WireSm::getChildren() const
    {
        std::vector<std::unique_ptr<Edge>>  edges = getEdges();
        std::vector<std::unique_ptr<Shape>> children;
        children.reserve(edges.size());
        for (size_t index = 0; index < edges.size(); index++)
        {
            children.push_back(std::move(edges[index]));
        }
        return children;
    }

    SmLoopuse* WireSm::getLoopuse() const
    {
        return loopuse;
    }
}

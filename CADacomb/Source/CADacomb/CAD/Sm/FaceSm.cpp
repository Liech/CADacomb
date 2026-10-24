#include "FaceSm.h"

#include "SmPyError.h"
#include "WireSm.h"
#include "measure.h"
#include <SmBSplineSurface.h>
#include <SmCone.h>
#include <SmCylinder.h>
#include <SmFace.h>
#include <SmLoopuse.h>
#include <SmOffsetSurface.h>
#include <SmPlane.h>
#include <SmSphere.h>
#include <SmSurfOfExtrusion.h>
#include <SmSurfOfRevolution.h>
#include <SmTorus.h>

namespace CADacomb::CAD
{
    FaceSm::FaceSm(const std::shared_ptr<SmBrep>& brepInput, SmFace* faceInput, Orientation orientationInput)
      : brep(brepInput)
      , face(faceInput)
      , orientation(orientationInput)
    {
    }

    FaceSm::~FaceSm() {}

    ShapeType FaceSm::getShapeType() const
    {
        return ShapeType::Face;
    }

    AABB3 FaceSm::getAABB() const
    {
        SmExtent3d box;
        SmStatus   status = face->CalculateBoundingBox(box, TRUE);
        if (status != SM_SUCCESS)
        {
            SmPyRaise(status, "SmFace::CalculateBoundingBox", __FILE__, __LINE__);
        }
        SmPoint3d minCorner = box.GetMin();
        SmPoint3d maxCorner = box.GetMax();
        return AABB3(glm::dvec3(minCorner.x, minCorner.y, minCorner.z), glm::dvec3(maxCorner.x, maxCorner.y, maxCorner.z));
    }

    double FaceSm::getSurfaceArea() const
    {
        smlibpy::FaceMeasure measure = smlibpy::face_measure(face, 1e-7);
        return measure.area;
    }

    SurfaceType FaceSm::getSurfaceType() const
    {
        SmSurface* surface = face->GetSurface();
        if (surface == nullptr)
        {
            return SurfaceType::Other;
        }
        switch (surface->GetType())
        {
            case SmPlane_TYPE:
                return SurfaceType::Plane;
            case SmCylinder_TYPE:
                return SurfaceType::Cylinder;
            case SmCone_TYPE:
            {
                const SmCone* cone = static_cast<const SmCone*>(surface);
                return (cone->IsCylinder() == TRUE) ? SurfaceType::Cylinder : SurfaceType::Cone;
            }
            case SmSphere_TYPE:
                return SurfaceType::Sphere;
            case SmTorus_TYPE:
                return SurfaceType::Torus;
            case SmSurfOfRevolution_TYPE:
                return SurfaceType::SurfaceOfRevolution;
            case SmSurfOfExtrusion_TYPE:
                return SurfaceType::SurfaceOfExtrusion;
            case SmBSplineSurface_TYPE:
            {
                const SmBSplineSurface* bspline = static_cast<const SmBSplineSurface*>(surface);
                switch (bspline->GetBSplineSurfaceForm())
                {
                    case SM_SF_PLANE_SURF:
                        return SurfaceType::Plane;
                    case SM_SF_CYLINDRICAL_SURF:
                        return SurfaceType::Cylinder;
                    case SM_SF_CONICAL_SURF:
                        return SurfaceType::Cone;
                    case SM_SF_SPHERICAL_SURF:
                        return SurfaceType::Sphere;
                    case SM_SF_SURF_OF_REVOLUTION:
                        return SurfaceType::SurfaceOfRevolution;
                    case SM_SF_SURF_OF_LINEAR_EXTRUSION:
                        return SurfaceType::SurfaceOfExtrusion;
                    default:
                        return SurfaceType::BSpline;
                }
            }
            case SmOffsetSurface_TYPE:
                return SurfaceType::OffsetSurface;
            default:
                break;
        }
        if (surface->IsKindOf(SmBSplineSurface_TYPE))
        {
            return SurfaceType::BSpline;
        }
        return SurfaceType::Other;
    }

    Orientation FaceSm::getOrientation() const
    {
        return orientation;
    }

    std::vector<std::unique_ptr<Wire>> FaceSm::getWires() const
    {
        std::vector<std::unique_ptr<Wire>> wires;
        SmTArray<SmLoopuse*>               loopuses;
        face->GetUpwardLoopuses(loopuses);
        wires.reserve(loopuses.GetSize());
        for (ULONG index = 0; index < loopuses.GetSize(); index++)
        {
            wires.push_back(std::make_unique<WireSm>(brep, loopuses[index], index == 0));
        }
        return wires;
    }

    std::vector<std::unique_ptr<Shape>> FaceSm::getChildren() const
    {
        std::vector<std::unique_ptr<Wire>>  wires = getWires();
        std::vector<std::unique_ptr<Shape>> children;
        children.reserve(wires.size());
        for (size_t index = 0; index < wires.size(); index++)
        {
            children.push_back(std::move(wires[index]));
        }
        return children;
    }

    SmFace* FaceSm::getFace() const
    {
        return face;
    }
}

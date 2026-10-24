#include "FaceSm.h"

#include "BrepTransformSm.h"
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

    void FaceSm::transform(const glm::dmat4& transformationMatrix)
    {
        BrepTransformSm::transform(brep.get(), transformationMatrix);
    }

    void FaceSm::translate(const glm::dvec3& translationVector)
    {
        BrepTransformSm::translate(brep.get(), translationVector);
    }

    void FaceSm::rotate(const glm::dvec3& axisOrigin, const glm::dvec3& axisDirection, double angleRadians)
    {
        BrepTransformSm::rotate(brep.get(), axisOrigin, axisDirection, angleRadians);
    }

    void FaceSm::scale(const glm::dvec3& centerPoint, double scaleFactor)
    {
        BrepTransformSm::scale(brep.get(), centerPoint, scaleFactor);
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

    std::pair<glm::dvec2, glm::dvec2> FaceSm::getUVBounds() const
    {
        SmExtent2d domain = face->GetUVDomain();
        return { glm::dvec2(domain.GetUMin(), domain.GetVMin()), glm::dvec2(domain.GetUMax(), domain.GetVMax()) };
    }

    glm::dvec3 FaceSm::evaluatePoint(double uParameter, double vParameter) const
    {
        SmSurface* surface = face->GetSurface();
        SmPoint3d  point;
        SmVector3d du;
        SmVector3d dv;
        SmStatus   status = surface->Evaluate1stDerivatives(SmPoint2d(uParameter, vParameter), TRUE, TRUE, point, du, dv);
        if (status != SM_SUCCESS)
        {
            SmPyRaise(status, "SmSurface::Evaluate1stDerivatives", __FILE__, __LINE__);
        }
        return glm::dvec3(point.x, point.y, point.z);
    }

    glm::dvec3 FaceSm::evaluateNormal(double uParameter, double vParameter) const
    {
        SmSurface* surface = face->GetSurface();
        SmVector3d normal;
        SmStatus   status = surface->EvaluateNormal(SmPoint2d(uParameter, vParameter), TRUE, TRUE, normal);
        if (status != SM_SUCCESS)
        {
            SmPyRaise(status, "SmSurface::EvaluateNormal", __FILE__, __LINE__);
        }
        glm::dvec3 result(normal.x, normal.y, normal.z);
        if (glm::length(result) > 1e-12)
        {
            result = glm::normalize(result);
        }
        if (orientation == Orientation::Reversed)
        {
            result = -result;
        }
        return result;
    }

    std::pair<glm::dvec3, glm::dvec3> FaceSm::evaluateTangents(double uParameter, double vParameter) const
    {
        SmSurface* surface = face->GetSurface();
        SmPoint3d  point;
        SmVector3d du;
        SmVector3d dv;
        SmStatus   status = surface->Evaluate1stDerivatives(SmPoint2d(uParameter, vParameter), TRUE, TRUE, point, du, dv);
        if (status != SM_SUCCESS)
        {
            SmPyRaise(status, "SmSurface::Evaluate1stDerivatives", __FILE__, __LINE__);
        }
        return { glm::dvec3(du.x, du.y, du.z), glm::dvec3(dv.x, dv.y, dv.z) };
    }
}

#include "SmlibSolid.h"

#include "SmPyError.h"
#include "ext_shared.h"
#include "measure.h"
#include <SmApiQueries.h>

namespace CADacomb::CAD
{
    SmlibSolid::SmlibSolid(SmBrep* brepInput)
      : brep(brepInput)
    {
    }

    SmlibSolid::~SmlibSolid()
    {
        if (brep != nullptr)
        {
            smlibpy::delete_brep(brep);
        }
    }

    ShapeType SmlibSolid::getShapeType() const
    {
        return ShapeType::Solid;
    }

    AABB3 SmlibSolid::getAABB() const
    {
        SmPoint3d minCorner;
        SmPoint3d maxCorner;
        SmStatus  status = SmApiBrepBoundingBox(brep, TRUE, minCorner, maxCorner);
        if (status != SM_SUCCESS)
        {
            SmPyRaise(status, "SmApiBrepBoundingBox", __FILE__, __LINE__);
        }
        return AABB3(glm::dvec3(minCorner.x, minCorner.y, minCorner.z), glm::dvec3(maxCorner.x, maxCorner.y, maxCorner.z));
    }

    std::string SmlibSolid::getName() const
    {
        return name;
    }

    void SmlibSolid::setName(const std::string& newName)
    {
        name = newName;
    }

    bool SmlibSolid::isManifold() const
    {
        SmBoolean manifoldFlag = FALSE;
        SmStatus  status       = SmApiBrepIsManifoldSolid(brep, manifoldFlag);
        if (status != SM_SUCCESS)
        {
            SmPyRaise(status, "SmApiBrepIsManifoldSolid", __FILE__, __LINE__);
        }
        return manifoldFlag != FALSE;
    }

    double SmlibSolid::getVolume() const
    {
        smlibpy::BrepMeasure measure = smlibpy::brep_measure(brep, 1e-7);
        return measure.volume;
    }

    double SmlibSolid::getSurfaceArea() const
    {
        smlibpy::BrepMeasure measure = smlibpy::brep_measure(brep, 1e-7);
        return measure.area;
    }

    std::vector<std::unique_ptr<Shell>> SmlibSolid::getShells() const
    {
        return std::vector<std::unique_ptr<Shell>>();
    }

    std::vector<std::unique_ptr<Shape>> SmlibSolid::getChildren() const
    {
        return std::vector<std::unique_ptr<Shape>>();
    }

    SmBrep* SmlibSolid::getBrep() const
    {
        return brep;
    }
}

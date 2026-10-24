#include "SolidSm.h"

#include "ShellSm.h"
#include "SmPyError.h"
#include "ext_shared.h"
#include "measure.h"
#include <SmApiQueries.h>
#include <SmBrep.h>
#include <SmRegion.h>
#include <SmShell.h>

namespace CADacomb::CAD
{
    SolidSm::SolidSm(SmBrep* brepInput)
      : brep(brepInput, &smlibpy::delete_brep)
    {
    }

    SolidSm::SolidSm(const std::shared_ptr<SmBrep>& brepInput)
      : brep(brepInput)
    {
    }

    SolidSm::~SolidSm() {}

    ShapeType SolidSm::getShapeType() const
    {
        return ShapeType::Solid;
    }

    AABB3 SolidSm::getAABB() const
    {
        SmPoint3d minCorner;
        SmPoint3d maxCorner;
        SmStatus  status = SmApiBrepBoundingBox(brep.get(), TRUE, minCorner, maxCorner);
        if (status != SM_SUCCESS)
        {
            SmPyRaise(status, "SmApiBrepBoundingBox", __FILE__, __LINE__);
        }
        return AABB3(glm::dvec3(minCorner.x, minCorner.y, minCorner.z), glm::dvec3(maxCorner.x, maxCorner.y, maxCorner.z));
    }

    std::string SolidSm::getName() const
    {
        return name;
    }

    void SolidSm::setName(const std::string& newName)
    {
        name = newName;
    }

    bool SolidSm::isManifold() const
    {
        SmBoolean manifoldFlag = FALSE;
        SmStatus  status       = SmApiBrepIsManifoldSolid(brep.get(), manifoldFlag);
        if (status != SM_SUCCESS)
        {
            SmPyRaise(status, "SmApiBrepIsManifoldSolid", __FILE__, __LINE__);
        }
        return manifoldFlag != FALSE;
    }

    double SolidSm::getVolume() const
    {
        smlibpy::BrepMeasure measure = smlibpy::brep_measure(brep.get(), 1e-7);
        return measure.volume;
    }

    double SolidSm::getSurfaceArea() const
    {
        smlibpy::BrepMeasure measure = smlibpy::brep_measure(brep.get(), 1e-7);
        return measure.area;
    }

    std::vector<std::unique_ptr<Shell>> SolidSm::getShells() const
    {
        std::vector<std::unique_ptr<Shell>> shells;
        SmTArray<SmRegion*>                 regions;
        brep->GetRegions(regions);
        SmRegion* infiniteRegion = brep->GetInfiniteRegion();

        for (ULONG regionIndex = 0; regionIndex < regions.GetSize(); regionIndex++)
        {
            SmRegion* region = regions[regionIndex];
            if (region == infiniteRegion)
            {
                continue;
            }
            SmTArray<SmShell*> regionShells;
            region->GetShells(regionShells);
            for (ULONG shellIndex = 0; shellIndex < regionShells.GetSize(); shellIndex++)
            {
                shells.push_back(std::make_unique<ShellSm>(brep, regionShells[shellIndex]));
            }
        }

        if (shells.empty() && infiniteRegion != nullptr)
        {
            SmTArray<SmShell*> infiniteShells;
            infiniteRegion->GetShells(infiniteShells);
            for (ULONG shellIndex = 0; shellIndex < infiniteShells.GetSize(); shellIndex++)
            {
                shells.push_back(std::make_unique<ShellSm>(brep, infiniteShells[shellIndex]));
            }
        }

        return shells;
    }

    std::vector<std::unique_ptr<Shape>> SolidSm::getChildren() const
    {
        std::vector<std::unique_ptr<Shell>> shells = getShells();
        std::vector<std::unique_ptr<Shape>> children;
        children.reserve(shells.size());
        for (size_t index = 0; index < shells.size(); index++)
        {
            children.push_back(std::move(shells[index]));
        }
        return children;
    }

    SmBrep* SolidSm::getBrep() const
    {
        return brep.get();
    }

    const std::shared_ptr<SmBrep>& SolidSm::getSharedBrep() const
    {
        return brep;
    }
}

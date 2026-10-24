#include "ShellSm.h"

#include "FaceSm.h"
#include "SmPyError.h"
#include <SmFaceuse.h>
#include <SmShell.h>

namespace CADacomb::CAD
{
    ShellSm::ShellSm(const std::shared_ptr<SmBrep>& brepInput, SmShell* shellInput)
      : brep(brepInput)
      , shell(shellInput)
    {
    }

    ShellSm::~ShellSm() {}

    ShapeType ShellSm::getShapeType() const
    {
        return ShapeType::Shell;
    }

    AABB3 ShellSm::getAABB() const
    {
        SmExtent3d box;
        SmStatus   status = shell->CalculateBoundingBox(box, TRUE);
        if (status != SM_SUCCESS)
        {
            SmPyRaise(status, "SmShell::CalculateBoundingBox", __FILE__, __LINE__);
        }
        SmPoint3d minCorner = box.GetMin();
        SmPoint3d maxCorner = box.GetMax();
        return AABB3(glm::dvec3(minCorner.x, minCorner.y, minCorner.z), glm::dvec3(maxCorner.x, maxCorner.y, maxCorner.z));
    }

    bool ShellSm::isClosed() const
    {
        return shell->IsClosed() != FALSE;
    }

    std::vector<std::unique_ptr<Face>> ShellSm::getFaces() const
    {
        std::vector<std::unique_ptr<Face>> faces;
        SmTArray<SmFaceuse*>               faceuses;
        shell->GetFaceuses(faceuses);
        faces.reserve(faceuses.GetSize());
        for (ULONG index = 0; index < faceuses.GetSize(); index++)
        {
            SmFaceuse*  faceuse     = faceuses[index];
            Orientation orientation = (faceuse->GetOrientation() == SM_OT_SAME) ? Orientation::Forward : Orientation::Reversed;
            faces.push_back(std::make_unique<FaceSm>(brep, faceuse->GetFace(), orientation));
        }
        return faces;
    }

    std::vector<std::unique_ptr<Shape>> ShellSm::getChildren() const
    {
        std::vector<std::unique_ptr<Face>>  faces = getFaces();
        std::vector<std::unique_ptr<Shape>> children;
        children.reserve(faces.size());
        for (size_t index = 0; index < faces.size(); index++)
        {
            children.push_back(std::move(faces[index]));
        }
        return children;
    }

    SmShell* ShellSm::getShell() const
    {
        return shell;
    }
}

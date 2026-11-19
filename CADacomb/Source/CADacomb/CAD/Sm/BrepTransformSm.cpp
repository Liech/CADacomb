#include "BrepTransformSm.h"

#include "SmPyError.h"
#include "ext_shared.h"
#include <SmApiGeneral.h>
#include <SmApiQueries.h>
#include <SmAxis2Placement.h>
#include <SmBrep.h>
#include <cmath>

namespace CADacomb::CAD
{
    BrepTransformSm::BrepTransformSm() {}

    BrepTransformSm::~BrepTransformSm() {}

    std::shared_ptr<SmBrep> BrepTransformSm::cloneBrep(const std::shared_ptr<SmBrep>& brep)
    {
        if (!brep)
        {
            return nullptr;
        }

        SmBrep*  copied = nullptr;
        SmStatus status = SmApiBrepCopy(brep.get(), copied);
        if (status != SM_SUCCESS)
        {
            SmPyRaise(status, "SmApiBrepCopy", __FILE__, __LINE__);
        }
        return std::shared_ptr<SmBrep>(copied, &smlibpy::delete_brep);
    }

    void BrepTransformSm::transform(SmBrep* brep, const glm::dmat4& transformationMatrix)
    {
        if (brep == nullptr)
        {
            return;
        }

        glm::dvec3 rawX(transformationMatrix[0].x, transformationMatrix[0].y, transformationMatrix[0].z);
        glm::dvec3 rawY(transformationMatrix[1].x, transformationMatrix[1].y, transformationMatrix[1].z);
        glm::dvec3 rawZ(transformationMatrix[2].x, transformationMatrix[2].y, transformationMatrix[2].z);
        double     scaleX = glm::length(rawX);
        double     scaleY = glm::length(rawY);
        double     scaleZ = glm::length(rawZ);

        if ((std::abs(scaleX - 1.0) > 1e-9 || std::abs(scaleY - 1.0) > 1e-9 || std::abs(scaleZ - 1.0) > 1e-9) && scaleX > 1e-12 && scaleY > 1e-12 && scaleZ > 1e-12)
        {
            SmStatus scaleStatus = SmApiScaleByPt(brep, SmVector3d(0.0, 0.0, 0.0), SmVector3d(scaleX, scaleY, scaleZ));
            if (scaleStatus != SM_SUCCESS)
            {
                SmPyRaise(scaleStatus, "SmApiScaleByPt", __FILE__, __LINE__);
            }
            rawX /= scaleX;
            rawY /= scaleY;
        }

        SmPoint3d        origin(transformationMatrix[3].x, transformationMatrix[3].y, transformationMatrix[3].z);
        SmVector3d       xAxis(rawX.x, rawX.y, rawX.z);
        SmVector3d       yAxis(rawY.x, rawY.y, rawY.z);
        SmAxis2Placement placement;
        placement.SetCanonical(origin, xAxis, yAxis);

        SmStatus status = SmApiTransform(brep, placement);
        if (status != SM_SUCCESS)
        {
            SmPyRaise(status, "SmApiTransform", __FILE__, __LINE__);
        }
    }

    void BrepTransformSm::translate(SmBrep* brep, const glm::dvec3& translationVector)
    {
        if (brep == nullptr)
        {
            return;
        }

        SmStatus status = SmApiTranslate(brep, SmVector3d(translationVector.x, translationVector.y, translationVector.z));
        if (status != SM_SUCCESS)
        {
            SmPyRaise(status, "SmApiTranslate", __FILE__, __LINE__);
        }
    }

    void BrepTransformSm::rotate(SmBrep* brep, const glm::dvec3& axisOrigin, const glm::dvec3& axisDirection, double angleRadians)
    {
        if (brep == nullptr)
        {
            return;
        }

        double   angleDegrees = glm::degrees(angleRadians);
        SmStatus status       = SmApiRotate(brep, SmVector3d(axisOrigin.x, axisOrigin.y, axisOrigin.z), SmVector3d(axisDirection.x, axisDirection.y, axisDirection.z), angleDegrees);
        if (status != SM_SUCCESS)
        {
            SmPyRaise(status, "SmApiRotate", __FILE__, __LINE__);
        }
    }

    void BrepTransformSm::scale(SmBrep* brep, const glm::dvec3& centerPoint, double scaleFactor)
    {
        if (brep == nullptr)
        {
            return;
        }

        SmStatus status = SmApiScaleByPt(brep, SmVector3d(centerPoint.x, centerPoint.y, centerPoint.z), SmVector3d(scaleFactor, scaleFactor, scaleFactor));
        if (status != SM_SUCCESS)
        {
            SmPyRaise(status, "SmApiScaleByPt", __FILE__, __LINE__);
        }
    }
}

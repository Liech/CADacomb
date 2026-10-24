#pragma once

#include "CADacomb/CAD/Interface/Shape.h"
#include "CADacomb/CAD/Step/StepArgument.h"
#include "CADacomb/CAD/Step/StepDocument.h"
#include "value.h"
#include <glm/glm.hpp>
#include <memory>
#include <string>
#include <vector>

class SmBrep;

namespace CADacomb::CAD
{
    class StepReaderSm
    {
      public:
        StepReaderSm();
        virtual ~StepReaderSm();

        std::vector<std::unique_ptr<Shape>> loadFile(const std::string& filePath) const;

      private:
        double                  detectLengthScale(const StepDocument& document) const;
        glm::dvec3              getPoint(const StepDocument& document, int entityId, double lengthScale) const;
        glm::dvec3              getDirection(const StepDocument& document, int entityId) const;
        void                    getAxis2Placement3d(const StepDocument& document, int entityId, double lengthScale, glm::dvec3& origin, glm::dvec3& zAxis, glm::dvec3& xAxis) const;
        std::vector<double>     expandKnots(const StepArgument& knotsArgument, const StepArgument& multiplicitiesArgument) const;
        double                  computeAngleInPlane(const glm::dvec3& origin, const glm::dvec3& zAxis, const glm::dvec3& xAxis, const glm::dvec3& point) const;
        glm::dvec3              findLargestAngularGap(const glm::dvec3& origin, const glm::dvec3& zAxis, const glm::dvec3& xAxis, const std::vector<glm::dvec3>& points) const;
        std::vector<glm::dvec3> sampleEdgeSpec(const smlibpy::Value& edgeSpec, int sampleCount) const;
        smlibpy::Value          buildEdgeSpec(const StepDocument& document, int edgeCurveId, double lengthScale) const;
        smlibpy::Value          buildSurfaceSpec(const StepDocument& document, int surfaceId, const smlibpy::Value& loops, const smlibpy::Value& edgesDictionary, double lengthScale) const;
        SmBrep*                 buildShell(const StepDocument& document, int shellId, bool isSolid, bool isReversed, double lengthScale) const;
        glm::dvec3              normalizeOrDefault(const glm::dvec3& vector) const;
        glm::dvec3              valueToVec3(const smlibpy::Value& value) const;
        smlibpy::Value          vec3ToValue(const glm::dvec3& vector) const;

        static const double piConstant;
    };
}

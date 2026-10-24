#pragma once

#include "CADacomb/CAD/Interface/Shape.h"
#include "CADacomb/CAD/Step/StepEntityBuffer.h"
#include "CADacomb/CAD/Step/StepSurfaceNormalInfo.h"
#include "value.h"
#include <SmTArray.h>
#include <glm/glm.hpp>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

class SmBrep;
class SmEdge;
class SmFace;

namespace CADacomb::CAD
{
    class StepWriterSm
    {
      public:
        StepWriterSm();
        virtual ~StepWriterSm();

        void writeShape(const Shape& shape, const std::string& filePath, const std::string& productName) const;
        void writeFile(const SmBrep* brep, const std::string& filePath, const std::string& productName) const;
        void writeBreps(const std::vector<const SmBrep*>& breps, const std::string& filePath, const std::string& productName) const;

      private:
        int                                   writeBSplineCurve(StepEntityBuffer& buffer, const smlibpy::Value& curveValue) const;
        std::pair<int, bool>                  writeCurve(StepEntityBuffer& buffer, const smlibpy::Value& dumpValue, const SmTArray<SmEdge*>& kernelEdges, int edgeIndex) const;
        int                                   writeBSplineSurface(StepEntityBuffer& buffer, const smlibpy::Value& surfaceValue) const;
        std::pair<int, StepSurfaceNormalInfo> writeSurface(StepEntityBuffer& buffer, const smlibpy::Value& dumpValue, int faceIndex) const;
        int                                   getOrCreateVertexEntity(StepEntityBuffer& buffer, const smlibpy::Value& dumpValue, std::unordered_map<int, int>& vertexEntityIds, int vertexIndex) const;
        int                                   getOrCreateEdgeEntity(StepEntityBuffer&             buffer,
                                                                    const smlibpy::Value&         dumpValue,
                                                                    const SmTArray<SmEdge*>&      kernelEdges,
                                                                    std::unordered_map<int, int>& vertexEntityIds,
                                                                    std::unordered_map<int, int>& edgeEntityIds,
                                                                    int                           edgeIndex) const;
        bool                                  evaluateSameSense(const SmTArray<SmFace*>& kernelFaces, int faceIndex, const smlibpy::Value& faceUse, const StepSurfaceNormalInfo& normalInfo) const;
        int                                   writeFace(StepEntityBuffer&             buffer,
                                                        const smlibpy::Value&         dumpValue,
                                                        const SmTArray<SmFace*>&      kernelFaces,
                                                        const SmTArray<SmEdge*>&      kernelEdges,
                                                        std::unordered_map<int, int>& vertexEntityIds,
                                                        std::unordered_map<int, int>& edgeEntityIds,
                                                        int                           faceIndex,
                                                        const smlibpy::Value&         faceUse) const;
        std::vector<int>                      writeSolids(StepEntityBuffer& buffer, const SmBrep* brep, const std::string& productName) const;
        glm::dvec3                            normalizeOrDefault(const glm::dvec3& vector) const;
        glm::dvec3                            valueToVec3(const smlibpy::Value& value) const;

        static const double piConstant;
    };
}

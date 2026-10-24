#pragma once

#include <glm/glm.hpp>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace CADacomb::CAD
{
    class StepEntityBuffer
    {
      public:
        StepEntityBuffer();
        virtual ~StepEntityBuffer();

        int                               addRawLine(const std::string& entityText);
        int                               addPoint(const glm::dvec3& point);
        int                               addDirection(const glm::dvec3& direction);
        int                               addAxis2Placement3d(const glm::dvec3& origin, const glm::dvec3& zAxis, const glm::dvec3& xAxis);
        std::pair<glm::dvec3, glm::dvec3> computeOrthonormalAxes(const glm::dvec3& zInput, const glm::dvec3& xInput) const;
        std::string                       formatReal(double value) const;
        std::string                       formatString(const std::string& text) const;
        std::string                       formatIntList(const std::vector<int>& values) const;
        std::string                       formatRealList(const std::vector<double>& values) const;
        void                              splitKnotMultiplicities(const std::vector<double>& knots, std::vector<double>& uniqueKnots, std::vector<int>& multiplicities) const;
        const std::vector<std::string>&   getLines() const;

      private:
        glm::dvec3 normalizeOrDefault(const glm::dvec3& vector) const;

        std::vector<std::string>             lines;
        std::unordered_map<std::string, int> points;
        std::unordered_map<std::string, int> directions;
    };
}

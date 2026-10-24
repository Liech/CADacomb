#include "StepEntityBuffer.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace CADacomb::CAD
{
    StepEntityBuffer::StepEntityBuffer() {}

    StepEntityBuffer::~StepEntityBuffer() {}

    int StepEntityBuffer::addRawLine(const std::string& entityText)
    {
        lines.push_back(entityText);
        return static_cast<int>(lines.size());
    }

    int StepEntityBuffer::addPoint(const glm::dvec3& point)
    {
        std::string                                          entityText = "CARTESIAN_POINT('',(" + formatReal(point.x) + "," + formatReal(point.y) + "," + formatReal(point.z) + "))";
        std::unordered_map<std::string, int>::const_iterator iterator   = points.find(entityText);
        if (iterator != points.end())
        {
            return iterator->second;
        }
        int entityId       = addRawLine(entityText);
        points[entityText] = entityId;
        return entityId;
    }

    int StepEntityBuffer::addDirection(const glm::dvec3& direction)
    {
        glm::dvec3                                           unitDirection = normalizeOrDefault(direction);
        std::string                                          entityText = "DIRECTION('',(" + formatReal(unitDirection.x) + "," + formatReal(unitDirection.y) + "," + formatReal(unitDirection.z) + "))";
        std::unordered_map<std::string, int>::const_iterator iterator   = directions.find(entityText);
        if (iterator != directions.end())
        {
            return iterator->second;
        }
        int entityId           = addRawLine(entityText);
        directions[entityText] = entityId;
        return entityId;
    }

    int StepEntityBuffer::addAxis2Placement3d(const glm::dvec3& origin, const glm::dvec3& zAxis, const glm::dvec3& xAxis)
    {
        return addRawLine("AXIS2_PLACEMENT_3D('',#" + std::to_string(addPoint(origin)) + ",#" + std::to_string(addDirection(zAxis)) + ",#" + std::to_string(addDirection(xAxis)) + ")");
    }

    std::pair<glm::dvec3, glm::dvec3> StepEntityBuffer::computeOrthonormalAxes(const glm::dvec3& zInput, const glm::dvec3& xInput) const
    {
        glm::dvec3 zAxis = normalizeOrDefault(zInput);
        glm::dvec3 xAxis = normalizeOrDefault(xInput - zAxis * glm::dot(xInput, zAxis));
        return std::make_pair(zAxis, xAxis);
    }

    std::string StepEntityBuffer::formatReal(double value) const
    {
        if (!std::isfinite(value))
        {
            throw std::invalid_argument("Non-finite number in STEP output");
        }
        if (value == 0.0)
        {
            return "0.";
        }
        std::ostringstream stream;
        stream << std::setprecision(16) << value;
        std::string formatted        = stream.str();
        size_t      exponentPosition = formatted.find_first_of("eE");
        if (exponentPosition != std::string::npos)
        {
            std::string mantissa = formatted.substr(0, exponentPosition);
            int         exponent = std::stoi(formatted.substr(exponentPosition + 1));
            if (mantissa.find('.') == std::string::npos)
            {
                mantissa += ".";
            }
            return mantissa + "E" + std::to_string(exponent);
        }
        if (formatted.find('.') == std::string::npos)
        {
            formatted += ".";
        }
        return formatted;
    }

    std::string StepEntityBuffer::formatString(const std::string& text) const
    {
        std::string formatted = "'";
        for (size_t index = 0; index < text.size(); index++)
        {
            char currentChar = text[index];
            if (currentChar == '\'')
            {
                formatted += "''";
            }
            else if (static_cast<unsigned char>(currentChar) < 128)
            {
                formatted += currentChar;
            }
            else
            {
                formatted += '?';
            }
        }
        formatted += "'";
        return formatted;
    }

    std::string StepEntityBuffer::formatIntList(const std::vector<int>& values) const
    {
        std::string formatted = "(";
        for (size_t index = 0; index < values.size(); index++)
        {
            if (index > 0)
            {
                formatted += ",";
            }
            formatted += std::to_string(values[index]);
        }
        formatted += ")";
        return formatted;
    }

    std::string StepEntityBuffer::formatRealList(const std::vector<double>& values) const
    {
        std::string formatted = "(";
        for (size_t index = 0; index < values.size(); index++)
        {
            if (index > 0)
            {
                formatted += ",";
            }
            formatted += formatReal(values[index]);
        }
        formatted += ")";
        return formatted;
    }

    void StepEntityBuffer::splitKnotMultiplicities(const std::vector<double>& knots, std::vector<double>& uniqueKnots, std::vector<int>& multiplicities) const
    {
        for (size_t index = 0; index < knots.size(); index++)
        {
            double knotValue = knots[index];
            if (!uniqueKnots.empty() && std::fabs(knotValue - uniqueKnots.back()) <= 1e-14 * std::max(1.0, std::fabs(knotValue)))
            {
                multiplicities.back() += 1;
            }
            else
            {
                uniqueKnots.push_back(knotValue);
                multiplicities.push_back(1);
            }
        }
    }

    const std::vector<std::string>& StepEntityBuffer::getLines() const
    {
        return lines;
    }

    glm::dvec3 StepEntityBuffer::normalizeOrDefault(const glm::dvec3& vector) const
    {
        double length = glm::length(vector);
        if (length > 1e-30)
        {
            return vector / length;
        }
        return glm::dvec3(0.0, 0.0, 1.0);
    }
}

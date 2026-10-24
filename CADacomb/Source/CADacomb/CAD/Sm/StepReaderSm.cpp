#include "StepReaderSm.h"

#include "CADacomb/CAD/Step/StepArgumentType.h"
#include "CADacomb/CAD/Step/StepParser.h"
#include "SmPyError.h"
#include "SolidSm.h"
#include "StepLoopEntrySm.h"
#include "ext_shared.h"
#include "step_build.h"
#include <SmApiBrep.h>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

namespace CADacomb::CAD
{
    const double StepReaderSm::piConstant = 3.14159265358979323846;

    StepReaderSm::StepReaderSm() {}

    StepReaderSm::~StepReaderSm() {}

    std::vector<std::unique_ptr<Shape>> StepReaderSm::loadFile(const std::string& filePath) const
    {
        std::ifstream inputStream(filePath, std::ios::binary);
        if (!inputStream)
        {
            throw std::runtime_error("Cannot open STEP file: " + filePath);
        }

        std::ostringstream bufferStream;
        bufferStream << inputStream.rdbuf();

        StepParser                          parser;
        StepDocument                        document    = parser.parse(bufferStream.str());
        double                              lengthScale = detectLengthScale(document);
        std::vector<std::unique_ptr<Shape>> shapes;

        const std::unordered_map<int, std::pair<std::string, std::vector<StepArgument>>>& entities = document.getEntities();
        for (std::unordered_map<int, std::pair<std::string, std::vector<StepArgument>>>::const_iterator iterator = entities.begin(); iterator != entities.end(); iterator++)
        {
            const std::string&               typeName  = iterator->second.first;
            const std::vector<StepArgument>& arguments = iterator->second.second;

            if (typeName == "MANIFOLD_SOLID_BREP")
            {
                SmBrep*                  brep      = buildShell(document, arguments[1].asReference(), true, false, lengthScale);
                std::unique_ptr<SolidSm> solid     = std::make_unique<SolidSm>(brep);
                std::string              solidName = arguments[0].asString();
                if (!solidName.empty())
                {
                    solid->setName(solidName);
                }
                shapes.push_back(std::move(solid));
            }
            else if (typeName == "BREP_WITH_VOIDS")
            {
                SmBrep*                          outerBody      = buildShell(document, arguments[1].asReference(), true, false, lengthScale);
                const std::vector<StepArgument>& voidReferences = arguments[2].asList();
                for (size_t voidIndex = 0; voidIndex < voidReferences.size(); voidIndex++)
                {
                    SmBrep*  cavityBody     = buildShell(document, voidReferences[voidIndex].asReference(), true, false, lengthScale);
                    SmBrep*  differenceBody = nullptr;
                    SmStatus status         = SmApiBooleanDifference(outerBody, cavityBody, differenceBody);
                    if (status != SM_SUCCESS)
                    {
                        SmPyRaise(status, "SmApiBooleanDifference", __FILE__, __LINE__);
                    }
                    outerBody = differenceBody;
                }
                std::unique_ptr<SolidSm> solid     = std::make_unique<SolidSm>(outerBody);
                std::string              solidName = arguments[0].asString();
                if (!solidName.empty())
                {
                    solid->setName(solidName);
                }
                shapes.push_back(std::move(solid));
            }
            else if (typeName == "SHELL_BASED_SURFACE_MODEL")
            {
                const std::vector<StepArgument>& shellReferences = arguments[1].asList();
                for (size_t shellIndex = 0; shellIndex < shellReferences.size(); shellIndex++)
                {
                    SmBrep* brep = buildShell(document, shellReferences[shellIndex].asReference(), false, false, lengthScale);
                    shapes.push_back(std::make_unique<SolidSm>(brep));
                }
            }
        }

        if (shapes.empty())
        {
            throw std::runtime_error("No B-rep solids or shells found in STEP file: " + filePath);
        }

        return shapes;
    }

    double StepReaderSm::detectLengthScale(const StepDocument& document) const
    {
        double                                                                           lengthScale     = 1.0;
        const std::unordered_map<int, std::map<std::string, std::vector<StepArgument>>>& complexEntities = document.getComplexEntities();
        std::set<int>                                                                    assignedUnitIds;

        for (std::unordered_map<int, std::map<std::string, std::vector<StepArgument>>>::const_iterator iterator = complexEntities.begin(); iterator != complexEntities.end(); iterator++)
        {
            const std::map<std::string, std::vector<StepArgument>>&          parts        = iterator->second;
            std::map<std::string, std::vector<StepArgument>>::const_iterator unitIterator = parts.find("GLOBAL_UNIT_ASSIGNED_CONTEXT");
            if (unitIterator != parts.end() && !unitIterator->second.empty() && unitIterator->second[0].getType() == StepArgumentType::List)
            {
                const std::vector<StepArgument>& unitReferences = unitIterator->second[0].asList();
                for (size_t index = 0; index < unitReferences.size(); index++)
                {
                    if (unitReferences[index].getType() == StepArgumentType::Reference)
                    {
                        assignedUnitIds.insert(unitReferences[index].asReference());
                    }
                }
            }
        }

        for (std::unordered_map<int, std::map<std::string, std::vector<StepArgument>>>::const_iterator iterator = complexEntities.begin(); iterator != complexEntities.end(); iterator++)
        {
            if (!assignedUnitIds.empty() && assignedUnitIds.count(iterator->first) == 0)
            {
                continue;
            }
            const std::map<std::string, std::vector<StepArgument>>&          parts      = iterator->second;
            std::map<std::string, std::vector<StepArgument>>::const_iterator siIterator = parts.find("SI_UNIT");
            if (siIterator != parts.end() && parts.count("LENGTH_UNIT") > 0 && !siIterator->second.empty())
            {
                std::string prefix = siIterator->second[0].asString();
                if (prefix.empty())
                {
                    lengthScale = 1000.0;
                }
                else if (prefix == "CENTI")
                {
                    lengthScale = 10.0;
                }
                else if (prefix == "MILLI")
                {
                    lengthScale = 1.0;
                }
                else if (prefix == "MICRO")
                {
                    lengthScale = 1e-3;
                }
            }
        }
        return lengthScale;
    }

    glm::dvec3 StepReaderSm::getPoint(const StepDocument& document, int entityId, double lengthScale) const
    {
        const std::vector<StepArgument>& arguments   = document.getEntityArguments(entityId);
        const std::vector<StepArgument>& coordinates = arguments[1].asList();
        double                           xValue      = coordinates.size() > 0 ? coordinates[0].asNumber() : 0.0;
        double                           yValue      = coordinates.size() > 1 ? coordinates[1].asNumber() : 0.0;
        double                           zValue      = coordinates.size() > 2 ? coordinates[2].asNumber() : 0.0;
        return glm::dvec3(xValue, yValue, zValue) * lengthScale;
    }

    glm::dvec3 StepReaderSm::getDirection(const StepDocument& document, int entityId) const
    {
        const std::string&               typeName  = document.getEntityType(entityId);
        const std::vector<StepArgument>& arguments = document.getEntityArguments(entityId);
        if (typeName == "VECTOR")
        {
            return getDirection(document, arguments[1].asReference());
        }
        const std::vector<StepArgument>& coordinates = arguments[1].asList();
        double                           xValue      = coordinates.size() > 0 ? coordinates[0].asNumber() : 0.0;
        double                           yValue      = coordinates.size() > 1 ? coordinates[1].asNumber() : 0.0;
        double                           zValue      = coordinates.size() > 2 ? coordinates[2].asNumber() : 0.0;
        return normalizeOrDefault(glm::dvec3(xValue, yValue, zValue));
    }

    void StepReaderSm::getAxis2Placement3d(const StepDocument& document, int entityId, double lengthScale, glm::dvec3& origin, glm::dvec3& zAxis, glm::dvec3& xAxis) const
    {
        const std::vector<StepArgument>& arguments = document.getEntityArguments(entityId);
        origin                                     = getPoint(document, arguments[1].asReference(), lengthScale);
        zAxis = (arguments.size() > 2 && arguments[2].getType() == StepArgumentType::Reference) ? getDirection(document, arguments[2].asReference()) : glm::dvec3(0.0, 0.0, 1.0);
        xAxis = (arguments.size() > 3 && arguments[3].getType() == StepArgumentType::Reference) ? getDirection(document, arguments[3].asReference())
                                                                                                : (std::fabs(zAxis.x) < 0.9 ? glm::dvec3(1.0, 0.0, 0.0) : glm::dvec3(0.0, 1.0, 0.0));
        xAxis = normalizeOrDefault(xAxis - zAxis * glm::dot(xAxis, zAxis));
    }

    std::vector<double> StepReaderSm::expandKnots(const StepArgument& knotsArgument, const StepArgument& multiplicitiesArgument) const
    {
        std::vector<double>              expandedKnots;
        const std::vector<StepArgument>& knotList         = knotsArgument.asList();
        const std::vector<StepArgument>& multiplicityList = multiplicitiesArgument.asList();
        for (size_t index = 0; index < knotList.size() && index < multiplicityList.size(); index++)
        {
            double knotValue    = knotList[index].asNumber();
            int    multiplicity = static_cast<int>(multiplicityList[index].asNumber());
            for (int repeatIndex = 0; repeatIndex < multiplicity; repeatIndex++)
            {
                expandedKnots.push_back(knotValue);
            }
        }
        return expandedKnots;
    }

    double StepReaderSm::computeAngleInPlane(const glm::dvec3& origin, const glm::dvec3& zAxis, const glm::dvec3& xAxis, const glm::dvec3& point) const
    {
        glm::dvec3 yAxis = glm::cross(zAxis, xAxis);
        glm::dvec3 delta = point - origin;
        return std::atan2(glm::dot(delta, yAxis), glm::dot(delta, xAxis));
    }

    glm::dvec3 StepReaderSm::findLargestAngularGap(const glm::dvec3& origin, const glm::dvec3& zAxis, const glm::dvec3& xAxis, const std::vector<glm::dvec3>& points) const
    {
        std::vector<double> angles;
        angles.reserve(points.size());
        for (size_t index = 0; index < points.size(); index++)
        {
            angles.push_back(computeAngleInPlane(origin, zAxis, xAxis, points[index]));
        }
        std::sort(angles.begin(), angles.end());
        double bestGap     = -1.0;
        double middleAngle = 0.0;
        for (size_t index = 0; index < angles.size(); index++)
        {
            double nextAngle = (index + 1 < angles.size()) ? angles[index + 1] : (angles[0] + 2.0 * piConstant);
            double gap       = nextAngle - angles[index];
            if (gap > bestGap)
            {
                bestGap     = gap;
                middleAngle = angles[index] + 0.5 * gap;
            }
        }
        glm::dvec3 yAxis = glm::cross(zAxis, xAxis);
        return normalizeOrDefault(xAxis * std::cos(middleAngle) + yAxis * std::sin(middleAngle));
    }

    std::vector<glm::dvec3> StepReaderSm::sampleEdgeSpec(const smlibpy::Value& edgeSpec, int sampleCount) const
    {
        std::string             kindName = edgeSpec["kind"].str();
        std::vector<glm::dvec3> sampledPoints;

        if (kindName == "line")
        {
            sampledPoints.push_back(valueToVec3(edgeSpec["p0"]));
            sampledPoints.push_back(valueToVec3(edgeSpec["p1"]));
            return sampledPoints;
        }
        if (kindName == "circle")
        {
            glm::dvec3 origin       = valueToVec3(edgeSpec["center"]);
            glm::dvec3 zAxis        = valueToVec3(edgeSpec["z"]);
            glm::dvec3 xAxis        = valueToVec3(edgeSpec["x"]);
            double     radius       = edgeSpec["radius"].num();
            double     sweepRadians = edgeSpec["sweep"].num() * piConstant / 180.0;
            glm::dvec3 yAxis        = glm::cross(zAxis, xAxis);
            for (int sampleIndex = 0; sampleIndex <= sampleCount; sampleIndex++)
            {
                double angle = sweepRadians * sampleIndex / sampleCount;
                sampledPoints.push_back(origin + (xAxis * std::cos(angle) + yAxis * std::sin(angle)) * radius);
            }
            return sampledPoints;
        }
        if (kindName == "ellipse")
        {
            glm::dvec3 origin         = valueToVec3(edgeSpec["center"]);
            glm::dvec3 zAxis          = valueToVec3(edgeSpec["z"]);
            glm::dvec3 xAxis          = valueToVec3(edgeSpec["x"]);
            double     radius1        = edgeSpec["r1"].num();
            double     radius2        = edgeSpec["r2"].num();
            glm::dvec3 yAxis          = glm::cross(zAxis, xAxis);
            glm::dvec3 point0         = valueToVec3(edgeSpec["p0"]);
            glm::dvec3 point1         = valueToVec3(edgeSpec["p1"]);
            glm::dvec3 delta0         = point0 - origin;
            glm::dvec3 delta1         = point1 - origin;
            double     startParameter = std::atan2(glm::dot(delta0, yAxis) / radius2, glm::dot(delta0, xAxis) / radius1);
            double     endParameter   = std::atan2(glm::dot(delta1, yAxis) / radius2, glm::dot(delta1, xAxis) / radius1);
            double     span           = edgeSpec["closed"].boolean() ? (2.0 * piConstant) : std::fmod(endParameter - startParameter + 4.0 * piConstant, 2.0 * piConstant);
            if (!edgeSpec["forward"].boolean())
            {
                span = (span > 0.0) ? (span - 2.0 * piConstant) : (-2.0 * piConstant);
            }
            for (int sampleIndex = 0; sampleIndex <= sampleCount; sampleIndex++)
            {
                double angle = startParameter + span * sampleIndex / sampleCount;
                sampledPoints.push_back(origin + xAxis * (radius1 * std::cos(angle)) + yAxis * (radius2 * std::sin(angle)));
            }
            return sampledPoints;
        }
        if (kindName == "bspline")
        {
            const std::vector<smlibpy::Value>& poles = edgeSpec["poles"].list();
            for (size_t index = 0; index < poles.size(); index++)
            {
                sampledPoints.push_back(valueToVec3(poles[index]));
            }
            return sampledPoints;
        }
        return sampledPoints;
    }

    smlibpy::Value StepReaderSm::buildEdgeSpec(const StepDocument& document, int edgeCurveId, double lengthScale) const
    {
        const std::vector<StepArgument>& edgeArguments = document.getEntityArguments(edgeCurveId);
        int                              vertex0Id     = edgeArguments[1].asReference();
        int                              vertex1Id     = edgeArguments[2].asReference();
        int                              curveId       = edgeArguments[3].asReference();
        bool                             sameSense     = (edgeArguments[4].asString() != "F");
        bool                             isClosed      = (vertex0Id == vertex1Id);

        glm::dvec3 point0 = getPoint(document, document.getEntityArguments(vertex0Id)[1].asReference(), lengthScale);
        glm::dvec3 point1 = getPoint(document, document.getEntityArguments(vertex1Id)[1].asReference(), lengthScale);

        while (true)
        {
            const std::string&               curveType      = document.getEntityType(curveId);
            const std::vector<StepArgument>& curveArguments = document.getEntityArguments(curveId);
            if (curveType == "SURFACE_CURVE" || curveType == "SEAM_CURVE" || curveType == "INTERSECTION_CURVE" || curveType == "TRIMMED_CURVE")
            {
                curveId = curveArguments[1].asReference();
            }
            else
            {
                break;
            }
        }

        const std::string&               curveType      = document.getEntityType(curveId);
        const std::vector<StepArgument>& curveArguments = document.getEntityArguments(curveId);
        smlibpy::Value                   edgeDictionary = smlibpy::Value::dict();

        if (curveType == "LINE")
        {
            edgeDictionary.set("kind", smlibpy::v("line"));
            edgeDictionary.set("p0", vec3ToValue(point0));
            edgeDictionary.set("p1", vec3ToValue(point1));
            return edgeDictionary;
        }

        if (curveType == "CIRCLE")
        {
            glm::dvec3 origin;
            glm::dvec3 zAxis;
            glm::dvec3 xAxis;
            getAxis2Placement3d(document, curveArguments[1].asReference(), lengthScale, origin, zAxis, xAxis);
            double radius = curveArguments[2].asNumber() * lengthScale;
            if (!sameSense)
            {
                zAxis = -zAxis;
            }
            glm::dvec3 startRadial = point0 - origin;
            startRadial            = normalizeOrDefault(startRadial - zAxis * glm::dot(startRadial, zAxis));
            double sweepDegrees    = 360.0;
            if (!isClosed)
            {
                double angleDegrees = std::fmod(computeAngleInPlane(origin, zAxis, startRadial, point1) * 180.0 / piConstant + 360.0, 360.0);
                sweepDegrees        = (angleDegrees > 1e-9) ? angleDegrees : 360.0;
            }
            edgeDictionary.set("kind", smlibpy::v("circle"));
            edgeDictionary.set("center", vec3ToValue(origin));
            edgeDictionary.set("z", vec3ToValue(zAxis));
            edgeDictionary.set("x", vec3ToValue(startRadial));
            edgeDictionary.set("radius", smlibpy::v(radius));
            edgeDictionary.set("sweep", smlibpy::v(sweepDegrees));
            return edgeDictionary;
        }

        if (curveType == "ELLIPSE")
        {
            glm::dvec3 origin;
            glm::dvec3 zAxis;
            glm::dvec3 xAxis;
            getAxis2Placement3d(document, curveArguments[1].asReference(), lengthScale, origin, zAxis, xAxis);
            edgeDictionary.set("kind", smlibpy::v("ellipse"));
            edgeDictionary.set("center", vec3ToValue(origin));
            edgeDictionary.set("z", vec3ToValue(zAxis));
            edgeDictionary.set("x", vec3ToValue(xAxis));
            edgeDictionary.set("r1", smlibpy::v(curveArguments[2].asNumber() * lengthScale));
            edgeDictionary.set("r2", smlibpy::v(curveArguments[3].asNumber() * lengthScale));
            edgeDictionary.set("p0", vec3ToValue(point0));
            edgeDictionary.set("p1", vec3ToValue(point1));
            edgeDictionary.set("forward", smlibpy::v(sameSense));
            edgeDictionary.set("closed", smlibpy::v(isClosed));
            return edgeDictionary;
        }

        int                       degree = 0;
        std::vector<StepArgument> poleReferences;
        std::vector<double>       knots;
        smlibpy::Value            weightsValue;

        if (curveType == "B_SPLINE_CURVE_WITH_KNOTS")
        {
            degree         = static_cast<int>(curveArguments[1].asNumber());
            poleReferences = curveArguments[2].asList();
            knots          = expandKnots(curveArguments[7], curveArguments[6]);
        }
        else if (document.hasComplexEntity(curveId))
        {
            const std::map<std::string, std::vector<StepArgument>>& parts            = document.getComplexEntity(curveId);
            const std::vector<StepArgument>&                        bsplineArguments = parts.at("B_SPLINE_CURVE");
            degree                                                                   = static_cast<int>(bsplineArguments[0].asNumber());
            poleReferences                                                           = bsplineArguments[1].asList();
            const std::vector<StepArgument>& knotArguments                           = parts.at("B_SPLINE_CURVE_WITH_KNOTS");
            knots                                                                    = expandKnots(knotArguments[1], knotArguments[0]);
            if (parts.count("RATIONAL_B_SPLINE_CURVE") > 0)
            {
                weightsValue                                = smlibpy::Value::make_list();
                const std::vector<StepArgument>& weightList = parts.at("RATIONAL_B_SPLINE_CURVE")[0].asList();
                for (size_t index = 0; index < weightList.size(); index++)
                {
                    weightsValue.push(smlibpy::v(weightList[index].asNumber()));
                }
            }
        }
        else
        {
            throw std::runtime_error("Unsupported STEP curve entity: " + curveType);
        }

        smlibpy::Value polesList = smlibpy::Value::make_list();
        for (size_t index = 0; index < poleReferences.size(); index++)
        {
            polesList.push(vec3ToValue(getPoint(document, poleReferences[index].asReference(), lengthScale)));
        }
        smlibpy::Value knotsList = smlibpy::Value::make_list();
        for (size_t index = 0; index < knots.size(); index++)
        {
            knotsList.push(smlibpy::v(knots[index]));
        }

        edgeDictionary.set("kind", smlibpy::v("bspline"));
        edgeDictionary.set("degree", smlibpy::v(degree));
        edgeDictionary.set("poles", polesList);
        edgeDictionary.set("knots", knotsList);
        edgeDictionary.set("weights", weightsValue);
        edgeDictionary.set("p0", vec3ToValue(point0));
        edgeDictionary.set("p1", vec3ToValue(point1));
        edgeDictionary.set("forward", smlibpy::v(sameSense));
        edgeDictionary.set("closed", smlibpy::v(isClosed));
        return edgeDictionary;
    }

    smlibpy::Value StepReaderSm::buildSurfaceSpec(const StepDocument& document, int surfaceId, const smlibpy::Value& loops, const smlibpy::Value& edgesDictionary, double lengthScale) const
    {
        const std::string&               surfaceType      = document.getEntityType(surfaceId);
        const std::vector<StepArgument>& surfaceArguments = document.getEntityArguments(surfaceId);
        std::vector<glm::dvec3>          sampledPoints;
        bool                             hasSeamLoop = false;
        int                              seamEdgeId  = -1;

        const std::vector<smlibpy::Value>& loopItems = loops.list();
        for (size_t loopIndex = 0; loopIndex < loopItems.size(); loopIndex++)
        {
            const smlibpy::Value& loopValue = loopItems[loopIndex];
            if (!loopValue.has("edges"))
            {
                continue;
            }
            std::unordered_map<int, int>       edgeUseCounts;
            const std::vector<smlibpy::Value>& edgeUses = loopValue["edges"].list();
            for (size_t useIndex = 0; useIndex < edgeUses.size(); useIndex++)
            {
                int edgeId = static_cast<int>(edgeUses[useIndex].at(0).num());
                edgeUseCounts[edgeId]++;
                std::vector<glm::dvec3> edgeSamples = sampleEdgeSpec(edgesDictionary[std::to_string(edgeId)], 16);
                for (size_t sampleIndex = 0; sampleIndex < edgeSamples.size(); sampleIndex++)
                {
                    sampledPoints.push_back(edgeSamples[sampleIndex]);
                }
            }
            for (std::unordered_map<int, int>::const_iterator iterator = edgeUseCounts.begin(); iterator != edgeUseCounts.end(); iterator++)
            {
                if (iterator->second > 1)
                {
                    hasSeamLoop = true;
                    seamEdgeId  = iterator->first;
                }
            }
        }

        smlibpy::Value surfaceSpec = smlibpy::Value::dict();
        if (surfaceType == "PLANE")
        {
            glm::dvec3 origin;
            glm::dvec3 zAxis;
            glm::dvec3 xAxis;
            getAxis2Placement3d(document, surfaceArguments[1].asReference(), lengthScale, origin, zAxis, xAxis);
            glm::dvec3 yAxis = glm::cross(zAxis, xAxis);
            double     uMin  = 1e30;
            double     uMax  = -1e30;
            double     vMin  = 1e30;
            double     vMax  = -1e30;
            for (size_t index = 0; index < sampledPoints.size(); index++)
            {
                glm::dvec3 delta  = sampledPoints[index] - origin;
                double     uCoord = glm::dot(delta, xAxis);
                double     vCoord = glm::dot(delta, yAxis);
                uMin              = std::min(uMin, uCoord);
                uMax              = std::max(uMax, uCoord);
                vMin              = std::min(vMin, vCoord);
                vMax              = std::max(vMax, vCoord);
            }
            double padding = 0.05 * std::max(std::max(uMax - uMin, vMax - vMin), 1e-6) + 1e-6;
            surfaceSpec.set("kind", smlibpy::v("plane"));
            surfaceSpec.set("origin", vec3ToValue(origin));
            surfaceSpec.set("z", vec3ToValue(zAxis));
            surfaceSpec.set("x", vec3ToValue(xAxis));
            surfaceSpec.set("domain", smlibpy::tup(uMin - padding, uMax + padding, vMin - padding, vMax + padding));
            return surfaceSpec;
        }

        if (surfaceType == "CYLINDRICAL_SURFACE" || surfaceType == "CONICAL_SURFACE" || surfaceType == "SPHERICAL_SURFACE" || surfaceType == "TOROIDAL_SURFACE" ||
            surfaceType == "DEGENERATE_TOROIDAL_SURFACE")
        {
            glm::dvec3 origin;
            glm::dvec3 zAxis;
            glm::dvec3 xAxis;
            getAxis2Placement3d(document, surfaceArguments[1].asReference(), lengthScale, origin, zAxis, xAxis);

            bool       foundSeamDirection = false;
            glm::dvec3 seamDirection(0.0, 0.0, 0.0);
            if (hasSeamLoop && seamEdgeId >= 0)
            {
                std::vector<glm::dvec3> radialDirections;
                std::vector<glm::dvec3> seamSamples = sampleEdgeSpec(edgesDictionary[std::to_string(seamEdgeId)], 8);
                for (size_t sampleIndex = 0; sampleIndex < seamSamples.size(); sampleIndex++)
                {
                    glm::dvec3 delta  = seamSamples[sampleIndex] - origin;
                    glm::dvec3 radial = delta - zAxis * glm::dot(delta, zAxis);
                    if (glm::length(radial) > 1e-9 * std::max(1.0, glm::length(delta)))
                    {
                        radialDirections.push_back(normalizeOrDefault(radial));
                    }
                }
                if (!radialDirections.empty())
                {
                    foundSeamDirection = true;
                    seamDirection      = radialDirections[radialDirections.size() / 2];
                }
            }

            if (foundSeamDirection)
            {
                xAxis = seamDirection;
            }
            else if (!sampledPoints.empty())
            {
                xAxis = findLargestAngularGap(origin, zAxis, xAxis, sampledPoints);
            }

            if (surfaceType == "CYLINDRICAL_SURFACE" || surfaceType == "CONICAL_SURFACE")
            {
                double radius    = surfaceArguments[2].asNumber() * lengthScale;
                double heightMin = 1e30;
                double heightMax = -1e30;
                for (size_t index = 0; index < sampledPoints.size(); index++)
                {
                    double height = glm::dot(sampledPoints[index] - origin, zAxis);
                    heightMin     = std::min(heightMin, height);
                    heightMax     = std::max(heightMax, height);
                }
                double padding    = 0.05 * std::max(heightMax - heightMin, 1e-6) + 1e-6;
                double lowHeight  = heightMin - padding;
                double highHeight = heightMax + padding;

                if (surfaceType == "CYLINDRICAL_SURFACE")
                {
                    surfaceSpec.set("kind", smlibpy::v("cylinder"));
                    surfaceSpec.set("origin", vec3ToValue(origin));
                    surfaceSpec.set("z", vec3ToValue(zAxis));
                    surfaceSpec.set("x", vec3ToValue(xAxis));
                    surfaceSpec.set("radius", smlibpy::v(radius));
                    surfaceSpec.set("height", smlibpy::tup(lowHeight, highHeight));
                    return surfaceSpec;
                }

                double semiAngle = surfaceArguments[3].asNumber();
                if (semiAngle < 0.0)
                {
                    semiAngle = -semiAngle;
                    zAxis     = -zAxis;
                    std::swap(lowHeight, highHeight);
                    lowHeight  = -lowHeight;
                    highHeight = -highHeight;
                }
                if (semiAngle > 1e-12)
                {
                    lowHeight = std::max(lowHeight, -radius / std::tan(semiAngle));
                }
                surfaceSpec.set("kind", smlibpy::v("cone"));
                surfaceSpec.set("origin", vec3ToValue(origin));
                surfaceSpec.set("z", vec3ToValue(zAxis));
                surfaceSpec.set("x", vec3ToValue(xAxis));
                surfaceSpec.set("radius", smlibpy::v(radius));
                surfaceSpec.set("semi_angle", smlibpy::v(semiAngle));
                surfaceSpec.set("height", smlibpy::tup(lowHeight, highHeight));
                return surfaceSpec;
            }

            if (surfaceType == "SPHERICAL_SURFACE")
            {
                surfaceSpec.set("kind", smlibpy::v("sphere"));
                surfaceSpec.set("origin", vec3ToValue(origin));
                surfaceSpec.set("z", vec3ToValue(zAxis));
                surfaceSpec.set("x", vec3ToValue(xAxis));
                surfaceSpec.set("radius", smlibpy::v(surfaceArguments[2].asNumber() * lengthScale));
                return surfaceSpec;
            }

            surfaceSpec.set("kind", smlibpy::v("torus"));
            surfaceSpec.set("origin", vec3ToValue(origin));
            surfaceSpec.set("z", vec3ToValue(zAxis));
            surfaceSpec.set("x", vec3ToValue(xAxis));
            surfaceSpec.set("major", smlibpy::v(surfaceArguments[2].asNumber() * lengthScale));
            surfaceSpec.set("minor", smlibpy::v(surfaceArguments[3].asNumber() * lengthScale));
            return surfaceSpec;
        }

        int                       uDegree = 0;
        int                       vDegree = 0;
        std::vector<StepArgument> controlGrid;
        std::vector<double>       uKnots;
        std::vector<double>       vKnots;
        smlibpy::Value            weightsValue;

        if (surfaceType == "B_SPLINE_SURFACE_WITH_KNOTS")
        {
            uDegree     = static_cast<int>(surfaceArguments[1].asNumber());
            vDegree     = static_cast<int>(surfaceArguments[2].asNumber());
            controlGrid = surfaceArguments[3].asList();
            uKnots      = expandKnots(surfaceArguments[10], surfaceArguments[8]);
            vKnots      = expandKnots(surfaceArguments[11], surfaceArguments[9]);
        }
        else if (document.hasComplexEntity(surfaceId))
        {
            const std::map<std::string, std::vector<StepArgument>>& parts            = document.getComplexEntity(surfaceId);
            const std::vector<StepArgument>&                        bsplineArguments = parts.at("B_SPLINE_SURFACE");
            uDegree                                                                  = static_cast<int>(bsplineArguments[0].asNumber());
            vDegree                                                                  = static_cast<int>(bsplineArguments[1].asNumber());
            controlGrid                                                              = bsplineArguments[2].asList();
            const std::vector<StepArgument>& knotArguments                           = parts.at("B_SPLINE_SURFACE_WITH_KNOTS");
            uKnots                                                                   = expandKnots(knotArguments[2], knotArguments[0]);
            vKnots                                                                   = expandKnots(knotArguments[3], knotArguments[1]);
            if (parts.count("RATIONAL_B_SPLINE_SURFACE") > 0)
            {
                weightsValue                                = smlibpy::Value::make_list();
                const std::vector<StepArgument>& weightRows = parts.at("RATIONAL_B_SPLINE_SURFACE")[0].asList();
                for (size_t rowIndex = 0; rowIndex < weightRows.size(); rowIndex++)
                {
                    const std::vector<StepArgument>& weightRow = weightRows[rowIndex].asList();
                    for (size_t columnIndex = 0; columnIndex < weightRow.size(); columnIndex++)
                    {
                        weightsValue.push(smlibpy::v(weightRow[columnIndex].asNumber()));
                    }
                }
            }
        }
        else
        {
            throw std::runtime_error("Unsupported STEP surface entity: " + surfaceType);
        }

        smlibpy::Value polesList = smlibpy::Value::make_list();
        for (size_t rowIndex = 0; rowIndex < controlGrid.size(); rowIndex++)
        {
            const std::vector<StepArgument>& rowItems = controlGrid[rowIndex].asList();
            for (size_t columnIndex = 0; columnIndex < rowItems.size(); columnIndex++)
            {
                polesList.push(vec3ToValue(getPoint(document, rowItems[columnIndex].asReference(), lengthScale)));
            }
        }
        smlibpy::Value uKnotsList = smlibpy::Value::make_list();
        for (size_t index = 0; index < uKnots.size(); index++)
        {
            uKnotsList.push(smlibpy::v(uKnots[index]));
        }
        smlibpy::Value vKnotsList = smlibpy::Value::make_list();
        for (size_t index = 0; index < vKnots.size(); index++)
        {
            vKnotsList.push(smlibpy::v(vKnots[index]));
        }

        surfaceSpec.set("kind", smlibpy::v("bspline"));
        surfaceSpec.set("u_degree", smlibpy::v(uDegree));
        surfaceSpec.set("v_degree", smlibpy::v(vDegree));
        surfaceSpec.set("poles", polesList);
        surfaceSpec.set("u_knots", uKnotsList);
        surfaceSpec.set("v_knots", vKnotsList);
        surfaceSpec.set("weights", weightsValue);
        return surfaceSpec;
    }

    SmBrep* StepReaderSm::buildShell(const StepDocument& document, int shellId, bool isSolid, bool isReversed, double lengthScale) const
    {
        const std::string&               shellType      = document.getEntityType(shellId);
        const std::vector<StepArgument>& shellArguments = document.getEntityArguments(shellId);
        if (shellType == "ORIENTED_CLOSED_SHELL")
        {
            bool orientationFlag = (shellArguments[3].asString() == "F");
            return buildShell(document, shellArguments[2].asReference(), isSolid, isReversed != orientationFlag, lengthScale);
        }

        smlibpy::Value               edgesDictionary = smlibpy::Value::dict();
        smlibpy::Value               facesList       = smlibpy::Value::make_list();
        std::unordered_map<int, int> edgeIndexMap;

        const std::vector<StepArgument>& faceReferences = shellArguments[1].asList();
        for (size_t faceIndex = 0; faceIndex < faceReferences.size(); faceIndex++)
        {
            const std::vector<StepArgument>& faceArguments = document.getEntityArguments(faceReferences[faceIndex].asReference());
            int                              surfaceId     = faceArguments[2].asReference();
            bool                             sameSense     = (faceArguments[3].asString() != "F");
            std::vector<StepLoopEntrySm>     outerLoops;
            std::vector<StepLoopEntrySm>     innerLoops;

            const std::vector<StepArgument>& boundReferences = faceArguments[1].asList();
            for (size_t boundIndex = 0; boundIndex < boundReferences.size(); boundIndex++)
            {
                int                              boundId        = boundReferences[boundIndex].asReference();
                const std::string&               boundType      = document.getEntityType(boundId);
                const std::vector<StepArgument>& boundArguments = document.getEntityArguments(boundId);
                bool                             isOuter        = (boundType == "FACE_OUTER_BOUND");
                bool                             boundSense     = (boundArguments[2].asString() != "F");
                int                              loopId         = boundArguments[1].asReference();
                const std::string&               loopType       = document.getEntityType(loopId);
                const std::vector<StepArgument>& loopArguments  = document.getEntityArguments(loopId);

                smlibpy::Value loopDictionary = smlibpy::Value::dict();
                if (loopType == "VERTEX_LOOP")
                {
                    int        vertexId    = loopArguments[1].asReference();
                    glm::dvec3 vertexPoint = getPoint(document, document.getEntityArguments(vertexId)[1].asReference(), lengthScale);
                    loopDictionary.set("vertex", vec3ToValue(vertexPoint));
                    if (isOuter)
                    {
                        outerLoops.push_back(StepLoopEntrySm(true, loopDictionary));
                    }
                    else
                    {
                        innerLoops.push_back(StepLoopEntrySm(false, loopDictionary));
                    }
                    continue;
                }

                std::vector<std::pair<int, bool>> orientedEdges;
                const std::vector<StepArgument>&  orientedEdgeReferences = loopArguments[1].asList();
                for (size_t edgeRefIndex = 0; edgeRefIndex < orientedEdgeReferences.size(); edgeRefIndex++)
                {
                    const std::vector<StepArgument>&             orientedEdgeArguments = document.getEntityArguments(orientedEdgeReferences[edgeRefIndex].asReference());
                    int                                          edgeCurveId           = orientedEdgeArguments[3].asReference();
                    bool                                         orientation           = (orientedEdgeArguments[4].asString() != "F");
                    std::unordered_map<int, int>::const_iterator mapIterator           = edgeIndexMap.find(edgeCurveId);
                    int                                          edgeId                = 0;
                    if (mapIterator == edgeIndexMap.end())
                    {
                        edgeId                    = static_cast<int>(edgeIndexMap.size());
                        edgeIndexMap[edgeCurveId] = edgeId;
                        edgesDictionary.set(std::to_string(edgeId), buildEdgeSpec(document, edgeCurveId, lengthScale));
                    }
                    else
                    {
                        edgeId = mapIterator->second;
                    }
                    orientedEdges.push_back(std::make_pair(edgeId, orientation));
                }

                if (!boundSense)
                {
                    std::reverse(orientedEdges.begin(), orientedEdges.end());
                    for (size_t itemIndex = 0; itemIndex < orientedEdges.size(); itemIndex++)
                    {
                        orientedEdges[itemIndex].second = !orientedEdges[itemIndex].second;
                    }
                }

                smlibpy::Value edgeListValue = smlibpy::Value::make_list();
                for (size_t itemIndex = 0; itemIndex < orientedEdges.size(); itemIndex++)
                {
                    edgeListValue.push(smlibpy::tup(orientedEdges[itemIndex].first, orientedEdges[itemIndex].second));
                }
                loopDictionary.set("edges", edgeListValue);
                if (isOuter)
                {
                    outerLoops.push_back(StepLoopEntrySm(true, loopDictionary));
                }
                else
                {
                    innerLoops.push_back(StepLoopEntrySm(false, loopDictionary));
                }
            }

            smlibpy::Value loopsValue = smlibpy::Value::make_list();
            for (size_t index = 0; index < outerLoops.size(); index++)
            {
                loopsValue.push(outerLoops[index].getLoopValue());
            }
            for (size_t index = 0; index < innerLoops.size(); index++)
            {
                loopsValue.push(innerLoops[index].getLoopValue());
            }

            smlibpy::Value surfaceSpec = buildSurfaceSpec(document, surfaceId, loopsValue, edgesDictionary, lengthScale);
            bool           freeSeam    = (surfaceSpec["kind"].str() == "bspline");

            smlibpy::Value faceDictionary = smlibpy::Value::dict();
            faceDictionary.set("surface", surfaceSpec);
            faceDictionary.set("same_sense", smlibpy::v(sameSense));
            faceDictionary.set("free_seam", smlibpy::v(freeSeam));
            faceDictionary.set("loops", loopsValue);
            facesList.push(faceDictionary);
        }

        bool    isClosed = isSolid && (shellType != "OPEN_SHELL");
        SmBrep* brep     = smlibpy::build_brep(facesList, edgesDictionary, isClosed, isReversed, false);

        smlibpy::Value invertedFaces = smlibpy::inverted_faces(brep);
        if (!invertedFaces.list().empty() && 2 * invertedFaces.list().size() < facesList.list().size())
        {
            std::set<long>                     invertedSet;
            const std::vector<smlibpy::Value>& invertedList = invertedFaces.list();
            for (size_t index = 0; index < invertedList.size(); index++)
            {
                invertedSet.insert(static_cast<long>(invertedList[index].num()));
            }
            smlibpy::Value                     fixedFaces    = smlibpy::Value::make_list();
            const std::vector<smlibpy::Value>& originalFaces = facesList.list();
            for (size_t index = 0; index < originalFaces.size(); index++)
            {
                smlibpy::Value faceCopy = originalFaces[index];
                if (invertedSet.count(static_cast<long>(index)) > 0)
                {
                    faceCopy.set("same_sense", smlibpy::v(!originalFaces[index]["same_sense"].boolean()));
                }
                fixedFaces.push(faceCopy);
            }
            smlibpy::delete_brep(brep);
            brep = smlibpy::build_brep(fixedFaces, edgesDictionary, isClosed, isReversed, false);
        }

        smlibpy::seam_problems(brep, true, nullptr);
        return brep;
    }

    glm::dvec3 StepReaderSm::normalizeOrDefault(const glm::dvec3& vector) const
    {
        double length = glm::length(vector);
        if (length > 1e-30)
        {
            return vector / length;
        }
        return glm::dvec3(0.0, 0.0, 1.0);
    }

    glm::dvec3 StepReaderSm::valueToVec3(const smlibpy::Value& value) const
    {
        std::vector<double> numbers = value.nums();
        if (numbers.size() >= 3)
        {
            return glm::dvec3(numbers[0], numbers[1], numbers[2]);
        }
        return glm::dvec3(0.0, 0.0, 0.0);
    }

    smlibpy::Value StepReaderSm::vec3ToValue(const glm::dvec3& vector) const
    {
        return smlibpy::tup(vector.x, vector.y, vector.z);
    }
}

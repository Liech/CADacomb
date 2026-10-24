#include "StepWriterSm.h"

#include "CompSolidSm.h"
#include "CompoundSm.h"
#include "SmPyError.h"
#include "SolidSm.h"
#include "ext_shared.h"
#include <SmApiQueries.h>
#include <SmApiSurfaces.h>
#include <SmBrep.h>
#include <SmEdge.h>
#include <SmFace.h>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <stdexcept>

namespace CADacomb::CAD
{
    const double StepWriterSm::piConstant = 3.14159265358979323846;

    StepWriterSm::StepWriterSm() {}

    StepWriterSm::~StepWriterSm() {}

    void StepWriterSm::writeShape(const Shape& shape, const std::string& filePath, const std::string& productName) const
    {
        if (shape.getShapeType() == ShapeType::Solid)
        {
            const SolidSm& solid = dynamic_cast<const SolidSm&>(shape);
            writeFile(solid.getBrep(), filePath, productName);
            return;
        }
        if (shape.getShapeType() == ShapeType::Compound)
        {
            const CompoundSm&                           compound   = dynamic_cast<const CompoundSm&>(shape);
            const std::vector<std::shared_ptr<SmBrep>>& sharedList = compound.getBreps();
            std::vector<const SmBrep*>                  breps;
            breps.reserve(sharedList.size());
            for (size_t index = 0; index < sharedList.size(); index++)
            {
                breps.push_back(sharedList[index].get());
            }
            writeBreps(breps, filePath, productName);
            return;
        }
        if (shape.getShapeType() == ShapeType::CompSolid)
        {
            const CompSolidSm&                          compSolid  = dynamic_cast<const CompSolidSm&>(shape);
            const std::vector<std::shared_ptr<SmBrep>>& sharedList = compSolid.getBreps();
            std::vector<const SmBrep*>                  breps;
            breps.reserve(sharedList.size());
            for (size_t index = 0; index < sharedList.size(); index++)
            {
                breps.push_back(sharedList[index].get());
            }
            writeBreps(breps, filePath, productName);
            return;
        }
        throw std::invalid_argument("Unsupported shape type for STEP export");
    }

    void StepWriterSm::writeFile(const SmBrep* brep, const std::string& filePath, const std::string& productName) const
    {
        std::vector<const SmBrep*> breps;
        breps.push_back(brep);
        writeBreps(breps, filePath, productName);
    }

    void StepWriterSm::writeBreps(const std::vector<const SmBrep*>& breps, const std::string& filePath, const std::string& productName) const
    {
        StepEntityBuffer buffer;
        int              applicationId = buffer.addRawLine("APPLICATION_CONTEXT('core data for automotive mechanical design processes')");
        buffer.addRawLine("APPLICATION_PROTOCOL_DEFINITION('international standard','automotive_design',2000,#" + std::to_string(applicationId) + ")");
        int productContextId = buffer.addRawLine("PRODUCT_CONTEXT('',#" + std::to_string(applicationId) + ",'mechanical')");
        int productId        = buffer.addRawLine("PRODUCT(" + buffer.formatString(productName) + "," + buffer.formatString(productName) + ",'',(#" + std::to_string(productContextId) + "))");
        buffer.addRawLine("PRODUCT_RELATED_PRODUCT_CATEGORY('part',$,(#" + std::to_string(productId) + "))");
        int formationId         = buffer.addRawLine("PRODUCT_DEFINITION_FORMATION('','',#" + std::to_string(productId) + ")");
        int definitionContextId = buffer.addRawLine("PRODUCT_DEFINITION_CONTEXT('part definition',#" + std::to_string(applicationId) + ",'design')");
        int productDefinitionId = buffer.addRawLine("PRODUCT_DEFINITION('design','',#" + std::to_string(formationId) + ",#" + std::to_string(definitionContextId) + ")");
        int productShapeId      = buffer.addRawLine("PRODUCT_DEFINITION_SHAPE('','',#" + std::to_string(productDefinitionId) + ")");

        int lengthUnitId       = buffer.addRawLine("(LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.))");
        int angleUnitId        = buffer.addRawLine("(NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.))");
        int solidAngleUnitId   = buffer.addRawLine("(NAMED_UNIT(*) SI_UNIT($,.STERADIAN.) SOLID_ANGLE_UNIT())");
        int uncertaintyId      = buffer.addRawLine("UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.E-07),#" + std::to_string(lengthUnitId) + ",'distance_accuracy_value','confusion accuracy')");
        int geometricContextId = buffer.addRawLine("(GEOMETRIC_REPRESENTATION_CONTEXT(3) GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#" + std::to_string(uncertaintyId) +
                                                   ")) GLOBAL_UNIT_ASSIGNED_CONTEXT((#" + std::to_string(lengthUnitId) + ",#" + std::to_string(angleUnitId) + ",#" + std::to_string(solidAngleUnitId) +
                                                   ")) REPRESENTATION_CONTEXT('Context #1','3D Context with UNIT and UNCERTAINTY'))");

        int         originPlacementId = buffer.addAxis2Placement3d(glm::dvec3(0.0, 0.0, 0.0), glm::dvec3(0.0, 0.0, 1.0), glm::dvec3(1.0, 0.0, 0.0));
        std::string itemsText         = "#" + std::to_string(originPlacementId);

        for (size_t brepIndex = 0; brepIndex < breps.size(); brepIndex++)
        {
            std::vector<int> solidIds = writeSolids(buffer, breps[brepIndex], productName);
            for (size_t index = 0; index < solidIds.size(); index++)
            {
                itemsText += ",#" + std::to_string(solidIds[index]);
            }
        }

        int representationId = buffer.addRawLine("ADVANCED_BREP_SHAPE_REPRESENTATION(" + buffer.formatString(productName) + ",(" + itemsText + "),#" + std::to_string(geometricContextId) + ")");
        buffer.addRawLine("SHAPE_DEFINITION_REPRESENTATION(#" + std::to_string(productShapeId) + ",#" + std::to_string(representationId) + ")");

        std::ofstream outputStream(filePath, std::ios::binary);
        if (!outputStream)
        {
            throw std::runtime_error("Cannot open STEP file for writing: " + filePath);
        }
        outputStream << "ISO-10303-21;\nHEADER;\n";
        outputStream << "FILE_DESCRIPTION(('CADacomb STEP export'),'2;1');\n";
        outputStream << "FILE_NAME(" << buffer.formatString(filePath) << ",'2026-01-01T00:00:00',(''),(''),'CADacomb','SMLib kernel','');\n";
        outputStream << "FILE_SCHEMA(('AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }'));\n";
        outputStream << "ENDSEC;\nDATA;\n";
        const std::vector<std::string>& lines = buffer.getLines();
        for (size_t index = 0; index < lines.size(); index++)
        {
            outputStream << "#" << (index + 1) << "=" << lines[index] << ";\n";
        }
        outputStream << "ENDSEC;\nEND-ISO-10303-21;\n";
    }

    int StepWriterSm::writeBSplineCurve(StepEntityBuffer& buffer, const smlibpy::Value& curveValue) const
    {
        std::string                        pointsText;
        const std::vector<smlibpy::Value>& poles = curveValue["poles"].list();
        for (size_t index = 0; index < poles.size(); index++)
        {
            if (index > 0)
            {
                pointsText += ",";
            }
            pointsText += "#" + std::to_string(buffer.addPoint(valueToVec3(poles[index])));
        }
        std::vector<double> uniqueKnots;
        std::vector<int>    multiplicities;
        buffer.splitKnotMultiplicities(curveValue["knots"].nums(), uniqueKnots, multiplicities);
        int degree = static_cast<int>(curveValue["degree"].num());
        if (curveValue["weights"].is_none())
        {
            return buffer.addRawLine("B_SPLINE_CURVE_WITH_KNOTS(''," + std::to_string(degree) + ",(" + pointsText + "),.UNSPECIFIED.,.F.,.F.," + buffer.formatIntList(multiplicities) + "," +
                                     buffer.formatRealList(uniqueKnots) + ",.UNSPECIFIED.)");
        }
        return buffer.addRawLine("(BOUNDED_CURVE() B_SPLINE_CURVE(" + std::to_string(degree) + ",(" + pointsText + "),.UNSPECIFIED.,.F.,.F.) B_SPLINE_CURVE_WITH_KNOTS(" +
                                 buffer.formatIntList(multiplicities) + "," + buffer.formatRealList(uniqueKnots) + ",.UNSPECIFIED.) CURVE() GEOMETRIC_REPRESENTATION_ITEM() RATIONAL_B_SPLINE_CURVE(" +
                                 buffer.formatRealList(curveValue["weights"].nums()) + ") REPRESENTATION_ITEM(''))");
    }

    std::pair<int, bool> StepWriterSm::writeCurve(StepEntityBuffer& buffer, const smlibpy::Value& dumpValue, const SmTArray<SmEdge*>& kernelEdges, int edgeIndex) const
    {
        const smlibpy::Value& edgeValue  = dumpValue["edges"].at(edgeIndex);
        const smlibpy::Value& curveValue = edgeValue["curve"];
        std::string           kindName   = curveValue["kind"].str();
        std::vector<double>   interval   = edgeValue["interval"].nums();
        double                t0         = interval[0];
        double                t1         = interval[1];

        if (kindName == "line")
        {
            SmCurve*   kernelCurve = kernelEdges[edgeIndex]->GetCurve();
            glm::dvec3 pointA      = valueToVec3(smlibpy::curve_eval(kernelCurve, t0, 0).at(0));
            glm::dvec3 pointB      = valueToVec3(smlibpy::curve_eval(kernelCurve, t1, 0).at(0));
            glm::dvec3 direction   = pointB - pointA;
            double     length      = glm::length(direction);
            int        vectorId    = buffer.addRawLine("VECTOR('',#" + std::to_string(buffer.addDirection(direction / length)) + "," + buffer.formatReal(length) + ")");
            int        lineId      = buffer.addRawLine("LINE('',#" + std::to_string(buffer.addPoint(pointA)) + ",#" + std::to_string(vectorId) + ")");
            return std::make_pair(lineId, true);
        }

        if (kindName == "circle" || kindName == "ellipse")
        {
            const smlibpy::Value&             placement = curveValue["placement"];
            glm::dvec3                        origin    = valueToVec3(placement["origin"]);
            std::pair<glm::dvec3, glm::dvec3> axes      = buffer.computeOrthonormalAxes(valueToVec3(placement["z_dir"]), valueToVec3(placement["x_dir"]));
            glm::dvec3                        zAxis     = axes.first;
            glm::dvec3                        xAxis     = axes.second;
            int                               axisId    = buffer.addAxis2Placement3d(origin, zAxis, xAxis);
            int                               entityId  = 0;

            if (kindName == "circle")
            {
                entityId = buffer.addRawLine("CIRCLE('',#" + std::to_string(axisId) + "," + buffer.formatReal(curveValue["radius"].num()) + ")");
            }
            else
            {
                double radius1 = curveValue["radius1"].num();
                double radius2 = curveValue["radius2"].num();
                if (radius1 >= radius2)
                {
                    entityId = buffer.addRawLine("ELLIPSE('',#" + std::to_string(axisId) + "," + buffer.formatReal(radius1) + "," + buffer.formatReal(radius2) + ")");
                }
                else
                {
                    glm::dvec3 yAxis = glm::cross(zAxis, xAxis);
                    axisId           = buffer.addAxis2Placement3d(origin, zAxis, yAxis);
                    entityId         = buffer.addRawLine("ELLIPSE('',#" + std::to_string(axisId) + "," + buffer.formatReal(radius2) + "," + buffer.formatReal(radius1) + ")");
                }
            }

            double         midParameter     = 0.5 * (t0 + t1);
            smlibpy::Value evaluated        = smlibpy::curve_eval(kernelEdges[edgeIndex]->GetCurve(), midParameter, 1);
            glm::dvec3     midPoint         = valueToVec3(evaluated.at(0));
            glm::dvec3     tangent          = valueToVec3(evaluated.at(1));
            glm::dvec3     counterClockwise = glm::cross(zAxis, midPoint - origin);
            return std::make_pair(entityId, glm::dot(tangent, counterClockwise) > 0.0);
        }

        return std::make_pair(writeBSplineCurve(buffer, curveValue), true);
    }

    int StepWriterSm::writeBSplineSurface(StepEntityBuffer& buffer, const smlibpy::Value& surfaceValue) const
    {
        std::vector<double> uUnique;
        std::vector<double> vUnique;
        std::vector<int>    uMultiplicities;
        std::vector<int>    vMultiplicities;
        std::vector<double> uKnots = surfaceValue["u_knots"].nums();
        std::vector<double> vKnots = surfaceValue["v_knots"].nums();
        buffer.splitKnotMultiplicities(uKnots, uUnique, uMultiplicities);
        buffer.splitKnotMultiplicities(vKnots, vUnique, vMultiplicities);

        int                                uDegree = static_cast<int>(surfaceValue["u_degree"].num());
        int                                vDegree = static_cast<int>(surfaceValue["v_degree"].num());
        int                                uCount  = static_cast<int>(uKnots.size()) - uDegree - 1;
        int                                vCount  = static_cast<int>(vKnots.size()) - vDegree - 1;
        const std::vector<smlibpy::Value>& poles   = surfaceValue["poles"].list();

        std::string controlNetText = "(";
        for (int uIndex = 0; uIndex < uCount; uIndex++)
        {
            if (uIndex > 0)
            {
                controlNetText += ",";
            }
            controlNetText += "(";
            for (int vIndex = 0; vIndex < vCount; vIndex++)
            {
                if (vIndex > 0)
                {
                    controlNetText += ",";
                }
                controlNetText += "#" + std::to_string(buffer.addPoint(valueToVec3(poles[uIndex * vCount + vIndex])));
            }
            controlNetText += ")";
        }
        controlNetText += ")";

        if (surfaceValue["weights"].is_none())
        {
            return buffer.addRawLine("B_SPLINE_SURFACE_WITH_KNOTS(''," + std::to_string(uDegree) + "," + std::to_string(vDegree) + "," + controlNetText + ",.UNSPECIFIED.,.F.,.F.,.F.," +
                                     buffer.formatIntList(uMultiplicities) + "," + buffer.formatIntList(vMultiplicities) + "," + buffer.formatRealList(uUnique) + "," + buffer.formatRealList(vUnique) +
                                     ",.UNSPECIFIED.)");
        }

        std::vector<double> weights        = surfaceValue["weights"].nums();
        std::string         weightRowsText = "(";
        for (int uIndex = 0; uIndex < uCount; uIndex++)
        {
            if (uIndex > 0)
            {
                weightRowsText += ",";
            }
            weightRowsText += "(";
            for (int vIndex = 0; vIndex < vCount; vIndex++)
            {
                if (vIndex > 0)
                {
                    weightRowsText += ",";
                }
                weightRowsText += buffer.formatReal(weights[uIndex * vCount + vIndex]);
            }
            weightRowsText += ")";
        }
        weightRowsText += ")";

        return buffer.addRawLine("(BOUNDED_SURFACE() B_SPLINE_SURFACE(" + std::to_string(uDegree) + "," + std::to_string(vDegree) + "," + controlNetText +
                                 ",.UNSPECIFIED.,.F.,.F.,.F.) B_SPLINE_SURFACE_WITH_KNOTS(" + buffer.formatIntList(uMultiplicities) + "," + buffer.formatIntList(vMultiplicities) + "," +
                                 buffer.formatRealList(uUnique) + "," + buffer.formatRealList(vUnique) + ",.UNSPECIFIED.) GEOMETRIC_REPRESENTATION_ITEM() RATIONAL_B_SPLINE_SURFACE(" + weightRowsText +
                                 ") REPRESENTATION_ITEM('') SURFACE())");
    }

    std::pair<int, StepSurfaceNormalInfo> StepWriterSm::writeSurface(StepEntityBuffer& buffer, const smlibpy::Value& dumpValue, int faceIndex) const
    {
        const smlibpy::Value& surfaceValue = dumpValue["faces"].at(faceIndex)["surface"];
        std::string           kindName     = surfaceValue["kind"].str();
        StepSurfaceNormalInfo normalInfo;

        if (kindName == "plane")
        {
            const smlibpy::Value&             placement = surfaceValue["placement"];
            glm::dvec3                        origin    = valueToVec3(placement["origin"]);
            std::pair<glm::dvec3, glm::dvec3> axes      = buffer.computeOrthonormalAxes(valueToVec3(placement["z_dir"]), valueToVec3(placement["x_dir"]));
            normalInfo.setKind("plane");
            normalInfo.setOrigin(origin);
            normalInfo.setZAxis(axes.first);
            int entityId = buffer.addRawLine("PLANE('',#" + std::to_string(buffer.addAxis2Placement3d(origin, axes.first, axes.second)) + ")");
            return std::make_pair(entityId, normalInfo);
        }

        if (kindName == "cylinder" || kindName == "cone")
        {
            const smlibpy::Value&             placement        = surfaceValue["placement"];
            glm::dvec3                        origin           = valueToVec3(placement["origin"]);
            std::pair<glm::dvec3, glm::dvec3> axes             = buffer.computeOrthonormalAxes(valueToVec3(placement["z_dir"]), valueToVec3(placement["x_dir"]));
            glm::dvec3                        zAxis            = axes.first;
            glm::dvec3                        xAxis            = axes.second;
            double                            semiAngleDegrees = surfaceValue.has("semi_angle_deg") ? surfaceValue["semi_angle_deg"].num() : 0.0;
            double                            radius           = surfaceValue["radius"].num();

            if (std::fabs(semiAngleDegrees) < 1e-12)
            {
                normalInfo.setKind("cylinder");
                normalInfo.setOrigin(origin);
                normalInfo.setZAxis(zAxis);
                normalInfo.setRadius(radius);
                int entityId = buffer.addRawLine("CYLINDRICAL_SURFACE('',#" + std::to_string(buffer.addAxis2Placement3d(origin, zAxis, xAxis)) + "," + buffer.formatReal(radius) + ")");
                return std::make_pair(entityId, normalInfo);
            }

            if (semiAngleDegrees < 0.0)
            {
                zAxis            = -zAxis;
                semiAngleDegrees = -semiAngleDegrees;
            }
            double semiAngleRadians = semiAngleDegrees * piConstant / 180.0;
            normalInfo.setKind("cone");
            normalInfo.setOrigin(origin);
            normalInfo.setZAxis(zAxis);
            normalInfo.setRadius(radius);
            normalInfo.setSemiAngleRad(semiAngleRadians);
            int entityId = buffer.addRawLine("CONICAL_SURFACE('',#" + std::to_string(buffer.addAxis2Placement3d(origin, zAxis, xAxis)) + "," + buffer.formatReal(radius) + "," +
                                             buffer.formatReal(semiAngleRadians) + ")");
            return std::make_pair(entityId, normalInfo);
        }

        if (kindName == "sphere")
        {
            const smlibpy::Value&             placement = surfaceValue["placement"];
            glm::dvec3                        origin    = valueToVec3(placement["origin"]);
            std::pair<glm::dvec3, glm::dvec3> axes      = buffer.computeOrthonormalAxes(valueToVec3(placement["z_dir"]), valueToVec3(placement["x_dir"]));
            double                            radius    = surfaceValue["radius"].num();
            normalInfo.setKind("sphere");
            normalInfo.setOrigin(origin);
            normalInfo.setZAxis(axes.first);
            normalInfo.setRadius(radius);
            int entityId = buffer.addRawLine("SPHERICAL_SURFACE('',#" + std::to_string(buffer.addAxis2Placement3d(origin, axes.first, axes.second)) + "," + buffer.formatReal(radius) + ")");
            return std::make_pair(entityId, normalInfo);
        }

        if (kindName == "torus")
        {
            const smlibpy::Value&             placement   = surfaceValue["placement"];
            glm::dvec3                        origin      = valueToVec3(placement["origin"]);
            std::pair<glm::dvec3, glm::dvec3> axes        = buffer.computeOrthonormalAxes(valueToVec3(placement["z_dir"]), valueToVec3(placement["x_dir"]));
            double                            majorRadius = surfaceValue["major_radius"].num();
            double                            minorRadius = surfaceValue["minor_radius"].num();
            normalInfo.setKind("torus");
            normalInfo.setOrigin(origin);
            normalInfo.setZAxis(axes.first);
            normalInfo.setRadius(minorRadius);
            normalInfo.setMajorRadius(majorRadius);
            int entityId = buffer.addRawLine("TOROIDAL_SURFACE('',#" + std::to_string(buffer.addAxis2Placement3d(origin, axes.first, axes.second)) + "," + buffer.formatReal(majorRadius) + "," +
                                             buffer.formatReal(minorRadius) + ")");
            return std::make_pair(entityId, normalInfo);
        }

        return std::make_pair(writeBSplineSurface(buffer, surfaceValue), normalInfo);
    }

    int StepWriterSm::getOrCreateVertexEntity(StepEntityBuffer& buffer, const smlibpy::Value& dumpValue, std::unordered_map<int, int>& vertexEntityIds, int vertexIndex) const
    {
        std::unordered_map<int, int>::const_iterator iterator = vertexEntityIds.find(vertexIndex);
        if (iterator != vertexEntityIds.end())
        {
            return iterator->second;
        }
        int entityId                 = buffer.addRawLine("VERTEX_POINT('',#" + std::to_string(buffer.addPoint(valueToVec3(dumpValue["vertices"].at(vertexIndex)))) + ")");
        vertexEntityIds[vertexIndex] = entityId;
        return entityId;
    }

    int StepWriterSm::getOrCreateEdgeEntity(StepEntityBuffer&             buffer,
                                            const smlibpy::Value&         dumpValue,
                                            const SmTArray<SmEdge*>&      kernelEdges,
                                            std::unordered_map<int, int>& vertexEntityIds,
                                            std::unordered_map<int, int>& edgeEntityIds,
                                            int                           edgeIndex) const
    {
        std::unordered_map<int, int>::const_iterator iterator = edgeEntityIds.find(edgeIndex);
        if (iterator != edgeEntityIds.end())
        {
            return iterator->second;
        }
        const smlibpy::Value& edgeValue   = dumpValue["edges"].at(edgeIndex);
        std::pair<int, bool>  curveResult = writeCurve(buffer, dumpValue, kernelEdges, edgeIndex);
        int                   vertex0Id   = getOrCreateVertexEntity(buffer, dumpValue, vertexEntityIds, static_cast<int>(edgeValue["v0"].num()));
        int                   vertex1Id   = getOrCreateVertexEntity(buffer, dumpValue, vertexEntityIds, static_cast<int>(edgeValue["v1"].num()));
        int                   entityId    = buffer.addRawLine("EDGE_CURVE('',#" + std::to_string(vertex0Id) + ",#" + std::to_string(vertex1Id) + ",#" + std::to_string(curveResult.first) + "," +
                                                              (curveResult.second ? ".T." : ".F.") + ")");
        edgeEntityIds[edgeIndex]          = entityId;
        return entityId;
    }

    bool StepWriterSm::evaluateSameSense(const SmTArray<SmFace*>& kernelFaces, int faceIndex, const smlibpy::Value& faceUse, const StepSurfaceNormalInfo& normalInfo) const
    {
        if (normalInfo.getKind().empty())
        {
            return faceUse["same"].boolean();
        }

        SmFace*   kernelFace = kernelFaces[faceIndex];
        SmPoint3d internalPoint;
        SmStatus  status = SmApiFaceInternalPoint(kernelFace, internalPoint);
        if (status != SM_SUCCESS)
        {
            SmPyRaise(status, "SmApiFaceInternalPoint", __FILE__, __LINE__);
        }

        SmPoint3d           closestPoint;
        SmTArray<SmPoint2d> uvParameters;
        double              distance = 0.0;
        status                       = SmApiSurfaceClosestPoint(kernelFace->GetSurface(), internalPoint, closestPoint, uvParameters, distance);
        if (status != SM_SUCCESS)
        {
            SmPyRaise(status, "SmApiSurfaceClosestPoint", __FILE__, __LINE__);
        }

        double     uParam = uvParameters[0].x;
        double     vParam = uvParameters[0].y;
        glm::dvec3 samplePoint(internalPoint.x, internalPoint.y, internalPoint.z);

        if (normalInfo.getKind() == "cone" || normalInfo.getKind() == "torus")
        {
            glm::dvec3 delta  = samplePoint - normalInfo.getOrigin();
            glm::dvec3 radial = delta - normalInfo.getZAxis() * glm::dot(delta, normalInfo.getZAxis());
            if (glm::length(radial) < 1e-9 * std::max(1.0, glm::length(delta)))
            {
                SmExtent2d domain = kernelFace->GetUVDomain();
                uParam            = 0.5 * (domain.GetMin().x + domain.GetMax().x);
                vParam            = 0.5 * (domain.GetMin().y + domain.GetMax().y);
                status            = SmApiEvaluateSurfacePoint(kernelFace->GetSurface(), SmVector2d(uParam, vParam), internalPoint);
                if (status != SM_SUCCESS)
                {
                    SmPyRaise(status, "SmApiEvaluateSurfacePoint", __FILE__, __LINE__);
                }
                samplePoint = glm::dvec3(internalPoint.x, internalPoint.y, internalPoint.z);
            }
        }

        SmVector3d kernelNormal;
        status = SmApiEvaluateSurfaceNormal(kernelFace->GetSurface(), SmVector2d(uParam, vParam), kernelNormal);
        if (status != SM_SUCCESS)
        {
            SmPyRaise(status, "SmApiEvaluateSurfaceNormal", __FILE__, __LINE__);
        }

        glm::dvec3 rawNormal(kernelNormal.x, kernelNormal.y, kernelNormal.z);
        glm::dvec3 faceNormal = faceUse["same"].boolean() ? rawNormal : -rawNormal;
        glm::dvec3 stepNormal(0.0, 0.0, 1.0);
        if (normalInfo.getKind() == "plane")
        {
            stepNormal = normalInfo.getZAxis();
        }
        else if (normalInfo.getKind() == "cylinder")
        {
            glm::dvec3 delta = samplePoint - normalInfo.getOrigin();
            stepNormal       = delta - normalInfo.getZAxis() * glm::dot(delta, normalInfo.getZAxis());
        }
        else if (normalInfo.getKind() == "cone")
        {
            glm::dvec3 delta  = samplePoint - normalInfo.getOrigin();
            glm::dvec3 radial = normalizeOrDefault(delta - normalInfo.getZAxis() * glm::dot(delta, normalInfo.getZAxis()));
            stepNormal        = radial * std::cos(normalInfo.getSemiAngleRad()) - normalInfo.getZAxis() * std::sin(normalInfo.getSemiAngleRad());
        }
        else if (normalInfo.getKind() == "sphere")
        {
            stepNormal = samplePoint - normalInfo.getOrigin();
        }
        else
        {
            glm::dvec3 delta  = samplePoint - normalInfo.getOrigin();
            glm::dvec3 radial = normalizeOrDefault(delta - normalInfo.getZAxis() * glm::dot(delta, normalInfo.getZAxis()));
            stepNormal        = samplePoint - (normalInfo.getOrigin() + radial * normalInfo.getMajorRadius());
        }

        return glm::dot(faceNormal, stepNormal) > 0.0;
    }

    int StepWriterSm::writeFace(StepEntityBuffer&             buffer,
                                const smlibpy::Value&         dumpValue,
                                const SmTArray<SmFace*>&      kernelFaces,
                                const SmTArray<SmEdge*>&      kernelEdges,
                                std::unordered_map<int, int>& vertexEntityIds,
                                std::unordered_map<int, int>& edgeEntityIds,
                                int                           faceIndex,
                                const smlibpy::Value&         faceUse) const
    {
        std::pair<int, StepSurfaceNormalInfo> surfaceResult = writeSurface(buffer, dumpValue, faceIndex);
        std::string                           boundsText;
        const std::vector<smlibpy::Value>&    loops = faceUse["loops"].list();

        for (size_t loopIndex = 0; loopIndex < loops.size(); loopIndex++)
        {
            const smlibpy::Value& loopValue    = loops[loopIndex];
            int                   loopEntityId = 0;
            if (!loopValue["vertex"].is_none() && loopValue["edges"].list().empty())
            {
                int vertexEntityId = getOrCreateVertexEntity(buffer, dumpValue, vertexEntityIds, static_cast<int>(loopValue["vertex"].num()));
                loopEntityId       = buffer.addRawLine("VERTEX_LOOP('',#" + std::to_string(vertexEntityId) + ")");
            }
            else
            {
                std::string                        orientedEdgesText;
                const std::vector<smlibpy::Value>& edgeItems = loopValue["edges"].list();
                for (size_t edgeItemIndex = 0; edgeItemIndex < edgeItems.size(); edgeItemIndex++)
                {
                    int  edgeIndex      = static_cast<int>(edgeItems[edgeItemIndex].at(0).num());
                    bool isForward      = edgeItems[edgeItemIndex].at(1).boolean();
                    int  edgeEntityId   = getOrCreateEdgeEntity(buffer, dumpValue, kernelEdges, vertexEntityIds, edgeEntityIds, edgeIndex);
                    int  orientedEdgeId = buffer.addRawLine("ORIENTED_EDGE('',*,*,#" + std::to_string(edgeEntityId) + "," + (isForward ? ".T." : ".F.") + ")");
                    if (edgeItemIndex > 0)
                    {
                        orientedEdgesText += ",";
                    }
                    orientedEdgesText += "#" + std::to_string(orientedEdgeId);
                }
                loopEntityId = buffer.addRawLine("EDGE_LOOP('',(" + orientedEdgesText + "))");
            }

            std::string boundKind = loopValue["outer"].boolean() ? "FACE_OUTER_BOUND" : "FACE_BOUND";
            int         boundId   = buffer.addRawLine(boundKind + "('',#" + std::to_string(loopEntityId) + ",.T.)");
            if (!boundsText.empty())
            {
                boundsText += ",";
            }
            boundsText += "#" + std::to_string(boundId);
        }

        bool sameSense = evaluateSameSense(kernelFaces, faceIndex, faceUse, surfaceResult.second);
        return buffer.addRawLine("ADVANCED_FACE('',(" + boundsText + "),#" + std::to_string(surfaceResult.first) + "," + (sameSense ? ".T." : ".F.") + ")");
    }

    std::vector<int> StepWriterSm::writeSolids(StepEntityBuffer& buffer, const SmBrep* brep, const std::string& productName) const
    {
        smlibpy::Value               dumpValue = smlibpy::brep_dump(brep, true);
        SmTArray<SmFace*>            kernelFaces;
        SmTArray<SmEdge*>            kernelEdges;
        std::unordered_map<int, int> vertexEntityIds;
        std::unordered_map<int, int> edgeEntityIds;

        brep->GetFaces(kernelFaces);
        brep->GetEdges(kernelEdges);

        std::vector<int>                                                     solidEntityIds;
        std::unordered_map<int, std::vector<std::pair<int, smlibpy::Value>>> shellFacesMap;
        const std::vector<smlibpy::Value>&                                   faceList = dumpValue["faces"].list();

        for (size_t faceIndex = 0; faceIndex < faceList.size(); faceIndex++)
        {
            const std::vector<smlibpy::Value>& uses = faceList[faceIndex]["uses"].list();
            for (int useIndex = 0; useIndex < 2; useIndex++)
            {
                if (uses[useIndex].is_none() || uses[useIndex]["shell"].is_none())
                {
                    continue;
                }
                int shellIndex = static_cast<int>(uses[useIndex]["shell"].num());
                shellFacesMap[shellIndex].push_back(std::make_pair(static_cast<int>(faceIndex), uses[1 - useIndex]));
            }
        }

        const std::vector<smlibpy::Value>& regionList = dumpValue["regions"].list();
        for (size_t regionIndex = 0; regionIndex < regionList.size(); regionIndex++)
        {
            const smlibpy::Value& regionValue = regionList[regionIndex];
            if (regionValue["infinite"].boolean() || regionValue["void"].boolean())
            {
                continue;
            }
            std::vector<int>                   shellEntityIds;
            const std::vector<smlibpy::Value>& regionShells = regionValue["shells"].list();
            for (size_t shellItemIndex = 0; shellItemIndex < regionShells.size(); shellItemIndex++)
            {
                int                                                shellIndex = static_cast<int>(regionShells[shellItemIndex].num());
                std::string                                        faceReferencesText;
                const std::vector<std::pair<int, smlibpy::Value>>& shellFaces = shellFacesMap[shellIndex];
                for (size_t itemIndex = 0; itemIndex < shellFaces.size(); itemIndex++)
                {
                    if (!faceReferencesText.empty())
                    {
                        faceReferencesText += ",";
                    }
                    int faceEntityId = writeFace(buffer, dumpValue, kernelFaces, kernelEdges, vertexEntityIds, edgeEntityIds, shellFaces[itemIndex].first, shellFaces[itemIndex].second);
                    faceReferencesText += "#" + std::to_string(faceEntityId);
                }
                shellEntityIds.push_back(buffer.addRawLine("CLOSED_SHELL('',(" + faceReferencesText + "))"));
            }

            if (shellEntityIds.empty())
            {
                continue;
            }
            if (shellEntityIds.size() == 1)
            {
                solidEntityIds.push_back(buffer.addRawLine("MANIFOLD_SOLID_BREP(" + buffer.formatString(productName) + ",#" + std::to_string(shellEntityIds[0]) + ")"));
            }
            else
            {
                std::string voidReferencesText;
                for (size_t shellItemIndex = 1; shellItemIndex < shellEntityIds.size(); shellItemIndex++)
                {
                    int orientedShellId = buffer.addRawLine("ORIENTED_CLOSED_SHELL('',*,#" + std::to_string(shellEntityIds[shellItemIndex]) + ",.F.)");
                    if (!voidReferencesText.empty())
                    {
                        voidReferencesText += ",";
                    }
                    voidReferencesText += "#" + std::to_string(orientedShellId);
                }
                solidEntityIds.push_back(buffer.addRawLine("BREP_WITH_VOIDS(" + buffer.formatString(productName) + ",#" + std::to_string(shellEntityIds[0]) + ",(" + voidReferencesText + "))"));
            }
        }

        return solidEntityIds;
    }

    glm::dvec3 StepWriterSm::normalizeOrDefault(const glm::dvec3& vector) const
    {
        double length = glm::length(vector);
        if (length > 1e-30)
        {
            return vector / length;
        }
        return glm::dvec3(0.0, 0.0, 1.0);
    }

    glm::dvec3 StepWriterSm::valueToVec3(const smlibpy::Value& value) const
    {
        std::vector<double> numbers = value.nums();
        if (numbers.size() >= 3)
        {
            return glm::dvec3(numbers[0], numbers[1], numbers[2]);
        }
        return glm::dvec3(0.0, 0.0, 0.0);
    }
}

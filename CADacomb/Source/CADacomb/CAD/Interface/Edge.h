#pragma once

#include "CurveType.h"
#include "Orientation.h"
#include "Shape.h"
#include "Vertex.h"
#include <memory>

namespace CADacomb::CAD
{
    class Edge : public Shape
    {
      public:
        Edge();
        virtual ~Edge();

        virtual CurveType   getCurveType() const   = 0;
        virtual Orientation getOrientation() const = 0;

        virtual std::unique_ptr<Vertex> getStart() const = 0;
        virtual std::unique_ptr<Vertex> getEnd() const   = 0;

        virtual double                    getLength() const                       = 0;
        virtual std::pair<double, double> getParameterRange() const               = 0;
        virtual glm::dvec3                evaluatePoint(double parameter) const   = 0;
        virtual glm::dvec3                evaluateTangent(double parameter) const = 0;
    };
}

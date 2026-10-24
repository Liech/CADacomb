#pragma once

#include "Shape.h"
#include "Shell.h"
#include <glm/glm.hpp>
#include <memory>
#include <string>
#include <vector>

namespace CADacomb::CAD
{
    class Solid : public Shape
    {
      public:
        Solid();
        virtual ~Solid();

        virtual std::string getName() const                  = 0;
        virtual void        setName(const std::string& name) = 0;
        virtual bool        isManifold() const               = 0;
        virtual double      getVolume() const                = 0;
        virtual double      getSurfaceArea() const           = 0;
        virtual glm::dvec3  getCenterOfMass() const          = 0;

        virtual std::vector<std::unique_ptr<Shell>> getShells() const = 0;
    };
}

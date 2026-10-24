#pragma once

#include "CADacomb/CAD/Interface/Solid.h"

class SmBrep;

namespace CADacomb::CAD
{
    class SmlibSolid : public Solid
    {
      public:
        SmlibSolid(SmBrep* brep);
        virtual ~SmlibSolid();

        ShapeType getShapeType() const override;
        AABB3     getAABB() const override;

        std::string getName() const override;
        void        setName(const std::string& name) override;
        bool        isManifold() const override;
        double      getVolume() const override;
        double      getSurfaceArea() const override;

        std::vector<std::unique_ptr<Shell>> getShells() const override;
        std::vector<std::unique_ptr<Shape>> getChildren() const override;

        SmBrep* getBrep() const;

      private:
        SmBrep*     brep = nullptr;
        std::string name = "SOLID";
    };
}

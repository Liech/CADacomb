#pragma once

#include "CADacomb/CAD/Interface/Solid.h"
#include <memory>
#include <string>

class SmBrep;

namespace CADacomb::CAD
{
    class SolidSm : public Solid
    {
      public:
        SolidSm(SmBrep* brepInput);
        SolidSm(const std::shared_ptr<SmBrep>& brepInput);
        virtual ~SolidSm();

        ShapeType getShapeType() const override;
        AABB3     getAABB() const override;

        std::string getName() const override;
        void        setName(const std::string& newName) override;
        bool        isManifold() const override;
        double      getVolume() const override;
        double      getSurfaceArea() const override;

        std::vector<std::unique_ptr<Shell>> getShells() const override;
        std::vector<std::unique_ptr<Shape>> getChildren() const override;

        SmBrep*                        getBrep() const;
        const std::shared_ptr<SmBrep>& getSharedBrep() const;

      private:
        std::shared_ptr<SmBrep> brep;
        std::string             name = "SOLID";
    };
}

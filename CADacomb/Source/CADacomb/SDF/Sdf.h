#pragma once

#include "CADacomb/Formula/Formula.h"
#include "CADacomb/Util/AABB3.h"

#include <nlohmann/json.hpp>
#include <string>

namespace CADacomb
{
    // Signed Distance Field defined by a 3D-to-1D Formula and local AABB bounds.
    class Sdf
    {
      public:
        Sdf();
        Sdf(const Formula& formula, const AABB3& aabb);
        Sdf(const std::string& expression, const AABB3& aabb);
        virtual ~Sdf();

        const Formula& getFormula() const;
        void           setFormula(const Formula& formula);

        const std::string& getExpression() const;
        void               setExpression(const std::string& expression);

        const AABB3& getAABB() const;
        void         setAABB(const AABB3& aabb);

        nlohmann::json toJson() const;
        static Sdf     fromJson(const nlohmann::json& json);

      private:
        Formula formula;
        AABB3   aabb;
    };
}

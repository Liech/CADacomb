#pragma once

#include "CADacomb/Formula/Formula.h"

#include <string>

namespace CADacomb
{
    // Emits a self-contained Lua snippet with a callable function generated from a Formula.
    class FormulaLuaEmitter
    {
      public:
        FormulaLuaEmitter();
        virtual ~FormulaLuaEmitter();

        static std::string emit(const Formula& formula);

      private:
        static std::string getMinimalPrelude();
        static std::string trim(const std::string& text);
    };
}

#pragma once

#include "CADacomb/Formula/Formula.h"

#include <string>

namespace CADacomb::Render
{
    // Emits a self-contained Slang snippet with a callable function generated from a Formula.
    class FormulaSlangEmitter
    {
      public:
        FormulaSlangEmitter();
        virtual ~FormulaSlangEmitter();

        static std::string emit(const Formula& formula);

      private:
        static std::string trim(const std::string& text);
    };
}

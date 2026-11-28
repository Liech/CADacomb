#pragma once

#include "CADacomb/Formula/Formula.h"
#include "Polyglot/Lua/LuaEngine.h"
#include "Polyglot/Node.h"

#include <memory>
#include <string>
#include <vector>

namespace CADacomb
{
    // Compiles and evaluates a float-array Formula on the CPU using Polyglot's LuaEngine.
    class FormulaLuaEvaluator
    {
      public:
        FormulaLuaEvaluator();
        virtual ~FormulaLuaEvaluator();

        bool               compile(const Formula& formula, std::string& outError);
        std::vector<float> evaluate(const std::vector<float>& input) const;
        bool               isCompiled() const;

      private:
        std::unique_ptr<Polyglot::Lua::LuaEngine> engine;
        Polyglot::Node                            evalFunction;
        size_t                                    outputCount = 0;
        bool                                      compiled    = false;
    };
}

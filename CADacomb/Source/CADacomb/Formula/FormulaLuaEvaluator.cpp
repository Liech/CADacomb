#include "CADacomb/Formula/FormulaLuaEvaluator.h"

#include "CADacomb/Formula/FormulaLuaEmitter.h"

#include <exception>

namespace CADacomb
{
    FormulaLuaEvaluator::FormulaLuaEvaluator() {}

    FormulaLuaEvaluator::~FormulaLuaEvaluator() {}

    bool FormulaLuaEvaluator::compile(const Formula& formula, std::string& outError)
    {
        compiled    = false;
        outputCount = formula.getOutputCount();
        outError.clear();

        try
        {
            engine                = std::make_unique<Polyglot::Lua::LuaEngine>();
            std::string luaSource = FormulaLuaEmitter::emit(formula);
            engine->executeString(luaSource);
            evalFunction = engine->getVar(formula.getName());
            if (!evalFunction.isFunction())
            {
                outError = "Emitted Lua script did not define function: " + formula.getName();
                return false;
            }
            compiled = true;
            return true;
        }
        catch (const std::exception& e)
        {
            outError = e.what();
            return false;
        }
    }

    std::vector<float> FormulaLuaEvaluator::evaluate(const std::vector<float>& input) const
    {
        std::vector<Polyglot::Node> inputNodes;
        inputNodes.reserve(input.size());
        for (size_t i = 0; i < input.size(); i++)
        {
            inputNodes.push_back(Polyglot::Node(static_cast<double>(input[i])));
        }

        Polyglot::Node fn         = evalFunction;
        Polyglot::Node resultNode = fn(Polyglot::Node(inputNodes));

        std::vector<float> result;
        result.reserve(outputCount);
        for (size_t i = 0; i < outputCount; i++)
        {
            result.push_back(static_cast<float>(resultNode[std::to_string(i + 1)].asDouble()));
        }
        return result;
    }

    bool FormulaLuaEvaluator::isCompiled() const
    {
        return compiled;
    }
}

#ifdef ISTESTPROJECT
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

TEST_CASE("FormulaLuaEvaluator/MultilineWithHelperAndDifferentArities")
{
    CADacomb::Formula formula;
    formula.setName("sphereAndRadius");
    formula.setInputCount(4);
    formula.setOutputCount(2);
    formula.setHelper("float sdSphere(float3 p, float r)\n"
                      "{\n"
                      "    return length(p) - r;\n"
                      "}");
    formula.setExpression("float3 p = float3(input[0], input[1], input[2]);\n"
                          "float r = input[3];\n"
                          "output[0] = sdSphere(p, r);\n"
                          "output[1] = length(p);");

    CADacomb::FormulaLuaEvaluator evaluator;
    std::string                   error;
    REQUIRE(evaluator.compile(formula, error));
    CHECK(error.empty());

    std::vector<float> out = evaluator.evaluate({ 3.0f, 4.0f, 0.0f, 2.0f });
    REQUIRE(out.size() == 2);
    CHECK(out[0] == Catch::Approx(3.0f));
    CHECK(out[1] == Catch::Approx(5.0f));
}

TEST_CASE("FormulaLuaEvaluator/InvalidFormulaReportsError")
{
    CADacomb::Formula formula;
    formula.setName("broken");
    formula.setExpression("output[0] = (((bad syntax;");

    CADacomb::FormulaLuaEvaluator evaluator;
    std::string                   error;
    CHECK_FALSE(evaluator.compile(formula, error));
    CHECK_FALSE(error.empty());
    CHECK_FALSE(evaluator.isCompiled());
}
#endif

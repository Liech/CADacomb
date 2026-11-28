#include "CADacombRender/Formula/FormulaSlangEmitter.h"

#include <sstream>

namespace CADacomb::Render
{
    FormulaSlangEmitter::FormulaSlangEmitter() {}

    FormulaSlangEmitter::~FormulaSlangEmitter() {}

    std::string FormulaSlangEmitter::emit(const Formula& formula)
    {
        std::ostringstream out;

        std::string helperCode = trim(formula.getHelper());
        if (!helperCode.empty())
        {
            out << helperCode << "\n\n";
        }

        out << "void " << formula.getName() << "(float input[" << formula.getInputCount() << "], out float output[" << formula.getOutputCount() << "])\n{\n";
        out << "    for (int _i = 0; _i < " << formula.getOutputCount() << "; _i++)\n";
        out << "    {\n";
        out << "        output[_i] = 0.0;\n";
        out << "    }\n";

        std::string exprCode = trim(formula.getExpression());
        if (!exprCode.empty())
        {
            std::istringstream lineStream(formula.getExpression());
            std::string        line;
            while (std::getline(lineStream, line))
            {
                out << "    " << line << "\n";
            }
        }

        out << "}\n";
        return out.str();
    }

    std::string FormulaSlangEmitter::trim(const std::string& text)
    {
        size_t start = 0;
        while (start < text.size() && (text[start] == ' ' || text[start] == '\t' || text[start] == '\r' || text[start] == '\n'))
        {
            start++;
        }
        size_t end = text.size();
        while (end > start && (text[end - 1] == ' ' || text[end - 1] == '\t' || text[end - 1] == '\r' || text[end - 1] == '\n'))
        {
            end--;
        }
        return text.substr(start, end - start);
    }
}

#ifdef ISTESTPROJECT
#include <catch2/catch_test_macros.hpp>

TEST_CASE("FormulaSlangEmitter/EmitsCallableSlangWithHelper")
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

    std::string slangSnippet = CADacomb::Render::FormulaSlangEmitter::emit(formula);
    CHECK(slangSnippet.find("float sdSphere(float3 p, float r)") != std::string::npos);
    CHECK(slangSnippet.find("void sphereAndRadius(float input[4], out float output[2])") != std::string::npos);
    CHECK(slangSnippet.find("output[0] = sdSphere(p, r);") != std::string::npos);
    CHECK(slangSnippet.find("output[1] = length(p);") != std::string::npos);
}
#endif

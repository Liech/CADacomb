#include "CADacomb/Formula/SlangToLuaTranspiler.h"

#include <sstream>
#include <vector>

namespace CADacomb
{
    SlangToLuaTranspiler::SlangToLuaTranspiler() {}

    SlangToLuaTranspiler::~SlangToLuaTranspiler() {}

    std::string SlangToLuaTranspiler::transpile(const std::string& slangCode)
    {
        std::string        cleaned = replaceBlockComments(slangCode);
        std::istringstream input(cleaned);
        std::ostringstream output;
        std::string        rawLine;
        bool               first = true;

        while (std::getline(input, rawLine))
        {
            std::string translated = translateLine(rawLine);
            if (!translated.empty())
            {
                if (!first)
                {
                    output << "\n";
                }
                output << translated;
                first = false;
            }
        }
        return output.str();
    }

    std::string SlangToLuaTranspiler::replaceBlockComments(const std::string& source)
    {
        std::string result;
        result.reserve(source.size());

        size_t i = 0;
        while (i < source.size())
        {
            if (i + 1 < source.size() && source[i] == '/' && source[i + 1] == '*')
            {
                result += "--[[";
                i += 2;
                while (i < source.size())
                {
                    if (i + 1 < source.size() && source[i] == '*' && source[i + 1] == '/')
                    {
                        result += "]]";
                        i += 2;
                        break;
                    }
                    result.push_back(source[i]);
                    i++;
                }
            }
            else
            {
                result.push_back(source[i]);
                i++;
            }
        }
        return result;
    }

    std::string SlangToLuaTranspiler::trim(const std::string& text)
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

    bool SlangToLuaTranspiler::isIdentifierChar(char c)
    {
        return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '_';
    }

    bool SlangToLuaTranspiler::isSlangTypeKeyword(const std::string& token)
    {
        return token == "float" || token == "float2" || token == "float3" || token == "float4" || token == "int" || token == "uint" || token == "bool" || token == "void";
    }

    std::string SlangToLuaTranspiler::replaceOperators(const std::string& line)
    {
        std::string result;
        result.reserve(line.size() + 16);

        size_t i = 0;
        while (i < line.size())
        {
            if (i + 1 < line.size() && line[i] == '/' && line[i + 1] == '/')
            {
                result += "--";
                result += line.substr(i + 2);
                break;
            }
            if (i + 1 < line.size() && line[i] == '!' && line[i + 1] == '=')
            {
                result += "~=";
                i += 2;
                continue;
            }
            if (i + 1 < line.size() && line[i] == '&' && line[i + 1] == '&')
            {
                if (!result.empty() && result.back() != ' ')
                {
                    result.push_back(' ');
                }
                result += "and";
                if (i + 2 < line.size() && line[i + 2] != ' ')
                {
                    result.push_back(' ');
                }
                i += 2;
                continue;
            }
            if (i + 1 < line.size() && line[i] == '|' && line[i + 1] == '|')
            {
                if (!result.empty() && result.back() != ' ')
                {
                    result.push_back(' ');
                }
                result += "or";
                if (i + 2 < line.size() && line[i + 2] != ' ')
                {
                    result.push_back(' ');
                }
                i += 2;
                continue;
            }
            result.push_back(line[i]);
            i++;
        }

        return result;
    }

    std::string SlangToLuaTranspiler::stripParameterTypes(const std::string& paramList)
    {
        std::string trimmed = trim(paramList);
        if (trimmed.empty())
        {
            return "";
        }

        std::ostringstream out;
        std::istringstream ss(trimmed);
        std::string        part;
        bool               first = true;
        while (std::getline(ss, part, ','))
        {
            std::string p = trim(part);
            if (p.empty())
            {
                continue;
            }
            size_t      lastSpace = p.find_last_of(" \t");
            std::string name      = (lastSpace != std::string::npos) ? trim(p.substr(lastSpace + 1)) : p;
            if (!first)
            {
                out << ", ";
            }
            out << name;
            first = false;
        }
        return out.str();
    }

    bool SlangToLuaTranspiler::tryTranslateFunctionHeader(const std::string& line, std::string& outLua)
    {
        std::string work = line;
        if (!work.empty() && work.back() == '{')
        {
            work.pop_back();
            work = trim(work);
        }

        size_t openParen  = work.find('(');
        size_t closeParen = work.rfind(')');
        if (openParen == std::string::npos || closeParen == std::string::npos || closeParen < openParen)
        {
            return false;
        }
        if (closeParen != work.size() - 1)
        {
            return false;
        }

        std::string beforeParen = trim(work.substr(0, openParen));
        size_t      spacePos    = beforeParen.find_first_of(" \t");
        if (spacePos == std::string::npos)
        {
            return false;
        }

        std::string returnType = trim(beforeParen.substr(0, spacePos));
        std::string funcName   = trim(beforeParen.substr(spacePos + 1));
        if (!isSlangTypeKeyword(returnType) || funcName.empty())
        {
            return false;
        }
        for (size_t i = 0; i < funcName.size(); i++)
        {
            if (!isIdentifierChar(funcName[i]))
            {
                return false;
            }
        }

        std::string params = stripParameterTypes(work.substr(openParen + 1, closeParen - openParen - 1));
        outLua             = "local function " + funcName + "(" + params + ")";
        return true;
    }

    bool SlangToLuaTranspiler::tryTranslateForLoop(const std::string& line, std::string& outLua)
    {
        if (line.rfind("for", 0) != 0 || line.size() <= 3 || isIdentifierChar(line[3]))
        {
            return false;
        }

        size_t openParen  = line.find('(');
        size_t closeParen = line.rfind(')');
        if (openParen == std::string::npos || closeParen == std::string::npos || closeParen <= openParen)
        {
            return false;
        }

        std::string header = line.substr(openParen + 1, closeParen - openParen - 1);
        size_t      semi1  = header.find(';');
        size_t      semi2  = (semi1 != std::string::npos) ? header.find(';', semi1 + 1) : std::string::npos;
        if (semi1 == std::string::npos || semi2 == std::string::npos)
        {
            return false;
        }

        std::string initPart = trim(header.substr(0, semi1));
        std::string condPart = trim(header.substr(semi1 + 1, semi2 - semi1 - 1));

        size_t eqPos = initPart.find('=');
        if (eqPos == std::string::npos)
        {
            return false;
        }

        std::string lhs       = trim(initPart.substr(0, eqPos));
        size_t      lastSpace = lhs.find_last_of(" \t");
        std::string varName   = (lastSpace != std::string::npos) ? trim(lhs.substr(lastSpace + 1)) : lhs;
        std::string startExpr = trim(initPart.substr(eqPos + 1));

        size_t ltPos = condPart.find('<');
        if (ltPos == std::string::npos)
        {
            return false;
        }
        bool        inclusive = (ltPos + 1 < condPart.size() && condPart[ltPos + 1] == '=');
        std::string endExpr   = trim(condPart.substr(ltPos + (inclusive ? 2 : 1)));

        outLua = "for " + varName + " = " + startExpr + ", " + (inclusive ? endExpr : "(" + endExpr + ") - 1") + " do";
        return true;
    }

    std::string SlangToLuaTranspiler::translateLine(const std::string& rawLine)
    {
        std::string line = replaceOperators(trim(rawLine));
        if (line.empty())
        {
            return "";
        }

        std::string forLua;
        if (tryTranslateForLoop(line, forLua))
        {
            return forLua;
        }

        if (!line.empty() && line.back() == ';')
        {
            line.pop_back();
            line = trim(line);
        }

        if (line == "{")
        {
            return "";
        }
        if (line == "}")
        {
            return "end";
        }

        if (line.rfind("} else if", 0) == 0)
        {
            std::string rest = trim(line.substr(9));
            if (!rest.empty() && rest.back() == '{')
            {
                rest.pop_back();
                rest = trim(rest);
            }
            return "elseif " + rest + " then";
        }
        if (line.rfind("else if", 0) == 0)
        {
            std::string rest = trim(line.substr(7));
            if (!rest.empty() && rest.back() == '{')
            {
                rest.pop_back();
                rest = trim(rest);
            }
            return "elseif " + rest + " then";
        }
        if (line == "} else {" || line == "} else" || line == "else {" || line == "else")
        {
            return "else";
        }
        if (line.rfind("if", 0) == 0 && line.size() > 2 && !isIdentifierChar(line[2]))
        {
            std::string rest = trim(line.substr(2));
            if (!rest.empty() && rest.back() == '{')
            {
                rest.pop_back();
                rest = trim(rest);
            }
            return "if " + rest + " then";
        }

        std::string funcLua;
        if (tryTranslateFunctionHeader(line, funcLua))
        {
            return funcLua;
        }

        if (line.rfind("const ", 0) == 0)
        {
            line = trim(line.substr(6));
        }

        const std::vector<std::string> typePrefixes = { "float4 ", "float3 ", "float2 ", "float ", "int ", "uint ", "bool " };
        for (size_t i = 0; i < typePrefixes.size(); i++)
        {
            if (line.rfind(typePrefixes[i], 0) == 0)
            {
                return "local " + trim(line.substr(typePrefixes[i].size()));
            }
        }

        return line;
    }
}

#ifdef ISTESTPROJECT
#include <catch2/catch_test_macros.hpp>

TEST_CASE("SlangToLuaTranspiler/FunctionsLocalsAndComments")
{
    std::string slang = "// sphere sdf\n"
                        "float sdSphere(float3 p, float r)\n"
                        "{\n"
                        "    const float d = length(p) - r;\n"
                        "    return d;\n"
                        "}";

    std::string lua = CADacomb::SlangToLuaTranspiler::transpile(slang);

    CHECK(lua.find("-- sphere sdf") != std::string::npos);
    CHECK(lua.find("local function sdSphere(p, r)") != std::string::npos);
    CHECK(lua.find("local d = length(p) - r") != std::string::npos);
    CHECK(lua.find("return d") != std::string::npos);
    CHECK(lua.find("end") != std::string::npos);
}

TEST_CASE("SlangToLuaTranspiler/ControlFlowAndOperators")
{
    std::string slang = "float sum = 0.0;\n"
                        "for (int i = 0; i < 4; i++)\n"
                        "{\n"
                        "    if (i != 2 && i >= 0)\n"
                        "    {\n"
                        "        sum = sum + i;\n"
                        "    }\n"
                        "    else\n"
                        "    {\n"
                        "        sum = sum + 1.0;\n"
                        "    }\n"
                        "}";

    std::string lua = CADacomb::SlangToLuaTranspiler::transpile(slang);

    CHECK(lua.find("local sum = 0.0") != std::string::npos);
    CHECK(lua.find("for i = 0, (4) - 1 do") != std::string::npos);
    CHECK(lua.find("if (i ~= 2 and i >= 0) then") != std::string::npos);
    CHECK(lua.find("else") != std::string::npos);
}
#endif

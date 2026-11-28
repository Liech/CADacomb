#pragma once

#include <string>

namespace CADacomb
{
    // General Slang-to-Lua transpiler for functions, locals, control flow, operators, and comments.
    class SlangToLuaTranspiler
    {
      public:
        SlangToLuaTranspiler();
        virtual ~SlangToLuaTranspiler();

        static std::string transpile(const std::string& slangCode);

      private:
        static std::string translateLine(const std::string& rawLine);
        static bool        tryTranslateFunctionHeader(const std::string& line, std::string& outLua);
        static bool        tryTranslateForLoop(const std::string& line, std::string& outLua);
        static std::string stripParameterTypes(const std::string& paramList);
        static std::string replaceBlockComments(const std::string& source);
        static std::string replaceOperators(const std::string& line);
        static std::string trim(const std::string& text);
        static bool        isIdentifierChar(char c);
        static bool        isSlangTypeKeyword(const std::string& token);
    };
}

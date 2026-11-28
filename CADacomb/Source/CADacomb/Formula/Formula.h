#pragma once

#include <cstddef>
#include <nlohmann/json.hpp>
#include <string>

namespace CADacomb
{
    // Portable Slang-dialect mathematical formula mapping an input float array to an output float array.
    class Formula
    {
      public:
        Formula();
        virtual ~Formula();

        const std::string& getName() const;
        void               setName(const std::string& name);

        const std::string& getExpression() const;
        void               setExpression(const std::string& expression);

        const std::string& getHelper() const;
        void               setHelper(const std::string& helper);

        size_t getInputCount() const;
        void   setInputCount(size_t inputCount);

        size_t getOutputCount() const;
        void   setOutputCount(size_t outputCount);

        nlohmann::json toJson() const;
        static Formula fromJson(const nlohmann::json& json);

      private:
        std::string name = "evalFormula";
        std::string expression;
        std::string helper;
        size_t      inputCount  = 1;
        size_t      outputCount = 1;
    };
}

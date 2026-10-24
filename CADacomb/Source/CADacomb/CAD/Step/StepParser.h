#pragma once

#include "StepArgument.h"
#include "StepDocument.h"
#include <cstddef>
#include <string>
#include <utility>
#include <vector>

namespace CADacomb::CAD
{
    class StepParser
    {
      public:
        StepParser();
        virtual ~StepParser();

        StepDocument parse(const std::string& rawText) const;

      private:
        std::string                                  stripComments(const std::string& rawText) const;
        void                                         skipWhitespace(const std::string& text, size_t& index) const;
        std::string                                  toUpperAscii(const std::string& inputText) const;
        std::pair<std::vector<StepArgument>, size_t> parseArguments(const std::string& text, size_t startPosition) const;
    };
}

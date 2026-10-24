#pragma once

#include "StepArgumentType.h"
#include <string>
#include <vector>

namespace CADacomb::CAD
{
    class StepArgument
    {
      public:
        StepArgument();
        virtual ~StepArgument();

        static StepArgument createNone();
        static StepArgument createStar();
        static StepArgument createReference(int referenceIdValue);
        static StepArgument createNumber(double number);
        static StepArgument createString(const std::string& text);
        static StepArgument createEnumeration(const std::string& enumText);
        static StepArgument createList(const std::vector<StepArgument>& items);
        static StepArgument createTyped(const std::string& typeName, const std::vector<StepArgument>& items);

        StepArgumentType                 getType() const;
        int                              asReference() const;
        double                           asNumber() const;
        std::string                      asString() const;
        const std::vector<StepArgument>& asList() const;

      private:
        StepArgumentType          type;
        int                       referenceId;
        double                    numberValue;
        std::string               stringValue;
        std::vector<StepArgument> listValue;
    };
}

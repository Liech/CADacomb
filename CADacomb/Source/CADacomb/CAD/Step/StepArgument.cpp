#include "StepArgument.h"

#include <stdexcept>

namespace CADacomb::CAD
{
    StepArgument::StepArgument()
      : type(StepArgumentType::None)
      , referenceId(0)
      , numberValue(0.0)
    {
    }

    StepArgument::~StepArgument() {}

    StepArgument StepArgument::createNone()
    {
        StepArgument argument;
        argument.type = StepArgumentType::None;
        return argument;
    }

    StepArgument StepArgument::createStar()
    {
        StepArgument argument;
        argument.type = StepArgumentType::Star;
        return argument;
    }

    StepArgument StepArgument::createReference(int referenceIdValue)
    {
        StepArgument argument;
        argument.type        = StepArgumentType::Reference;
        argument.referenceId = referenceIdValue;
        return argument;
    }

    StepArgument StepArgument::createNumber(double number)
    {
        StepArgument argument;
        argument.type        = StepArgumentType::Number;
        argument.numberValue = number;
        return argument;
    }

    StepArgument StepArgument::createString(const std::string& text)
    {
        StepArgument argument;
        argument.type        = StepArgumentType::String;
        argument.stringValue = text;
        return argument;
    }

    StepArgument StepArgument::createEnumeration(const std::string& enumText)
    {
        StepArgument argument;
        argument.type        = StepArgumentType::Enumeration;
        argument.stringValue = enumText;
        return argument;
    }

    StepArgument StepArgument::createList(const std::vector<StepArgument>& items)
    {
        StepArgument argument;
        argument.type      = StepArgumentType::List;
        argument.listValue = items;
        return argument;
    }

    StepArgument StepArgument::createTyped(const std::string& typeName, const std::vector<StepArgument>& items)
    {
        StepArgument argument;
        argument.type        = StepArgumentType::Typed;
        argument.stringValue = typeName;
        argument.listValue   = items;
        return argument;
    }

    StepArgumentType StepArgument::getType() const
    {
        return type;
    }

    int StepArgument::asReference() const
    {
        if (type != StepArgumentType::Reference)
        {
            throw std::runtime_error("Expected STEP entity reference");
        }
        return referenceId;
    }

    double StepArgument::asNumber() const
    {
        if (type == StepArgumentType::Number)
        {
            return numberValue;
        }
        if (type == StepArgumentType::Typed && !listValue.empty())
        {
            return listValue[0].asNumber();
        }
        throw std::runtime_error("Expected STEP numeric argument");
    }

    std::string StepArgument::asString() const
    {
        if (type == StepArgumentType::String || type == StepArgumentType::Enumeration)
        {
            return stringValue;
        }
        return "";
    }

    const std::vector<StepArgument>& StepArgument::asList() const
    {
        if (type != StepArgumentType::List)
        {
            throw std::runtime_error("Expected STEP list argument");
        }
        return listValue;
    }
}

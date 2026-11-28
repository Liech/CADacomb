#include "CADacomb/Formula/Formula.h"

#include <stdexcept>

namespace CADacomb
{
    Formula::Formula() {}

    Formula::~Formula() {}

    const std::string& Formula::getName() const
    {
        return name;
    }

    void Formula::setName(const std::string& name)
    {
        this->name = name;
    }

    const std::string& Formula::getExpression() const
    {
        return expression;
    }

    void Formula::setExpression(const std::string& expression)
    {
        this->expression = expression;
    }

    const std::string& Formula::getHelper() const
    {
        return helper;
    }

    void Formula::setHelper(const std::string& helper)
    {
        this->helper = helper;
    }

    size_t Formula::getInputCount() const
    {
        return inputCount;
    }

    void Formula::setInputCount(size_t inputCount)
    {
        this->inputCount = inputCount;
    }

    size_t Formula::getOutputCount() const
    {
        return outputCount;
    }

    void Formula::setOutputCount(size_t outputCount)
    {
        this->outputCount = outputCount;
    }

    nlohmann::json Formula::toJson() const
    {
        nlohmann::json json;
        json["Version"]     = 1;
        json["Name"]        = name;
        json["Expression"]  = expression;
        json["Helper"]      = helper;
        json["InputCount"]  = inputCount;
        json["OutputCount"] = outputCount;
        return json;
    }

    Formula Formula::fromJson(const nlohmann::json& json)
    {
        int version = json.at("Version").get<int>();
        if (version != 1)
        {
            throw std::runtime_error("Unsupported Formula version");
        }

        Formula formula;
        formula.setName(json.at("Name").get<std::string>());
        formula.setExpression(json.at("Expression").get<std::string>());
        formula.setHelper(json.at("Helper").get<std::string>());
        formula.setInputCount(json.at("InputCount").get<size_t>());
        formula.setOutputCount(json.at("OutputCount").get<size_t>());
        return formula;
    }
}

#ifdef ISTESTPROJECT
#include <catch2/catch_test_macros.hpp>

TEST_CASE("Formula/JsonRoundTrip")
{
    CADacomb::Formula formula;
    formula.setName("sphereSdf");
    formula.setHelper("float sdSphere(float3 p, float r)\n{\n    return length(p) - r;\n}");
    formula.setExpression("float3 p = float3(input[0], input[1], input[2]);\noutput[0] = sdSphere(p, input[3]);");
    formula.setInputCount(4);
    formula.setOutputCount(1);

    nlohmann::json    json   = formula.toJson();
    CADacomb::Formula loaded = CADacomb::Formula::fromJson(json);

    CHECK(json.at("Version").get<int>() == 1);
    CHECK(loaded.getName() == "sphereSdf");
    CHECK(loaded.getHelper() == formula.getHelper());
    CHECK(loaded.getExpression() == formula.getExpression());
    CHECK(loaded.getInputCount() == 4);
    CHECK(loaded.getOutputCount() == 1);
}
#endif

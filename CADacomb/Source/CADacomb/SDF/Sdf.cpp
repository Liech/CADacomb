#include "CADacomb/SDF/Sdf.h"

#include <stdexcept>

namespace CADacomb
{
    Sdf::Sdf()
      : aabb(glm::dvec3(-1.0), glm::dvec3(1.0))
    {
        formula.setName("sdfFormula");
        formula.setInputCount(3);
        formula.setOutputCount(1);
        formula.setExpression("float3 p = float3(input[0], input[1], input[2]);\noutput[0] = length(p) - 0.5;");
    }

    Sdf::Sdf(const Formula& formula, const AABB3& aabb)
      : formula(formula)
      , aabb(aabb)
    {
    }

    Sdf::Sdf(const std::string& expression, const AABB3& aabb)
      : aabb(aabb)
    {
        formula.setName("sdfFormula");
        formula.setInputCount(3);
        formula.setOutputCount(1);
        formula.setExpression(expression);
    }

    Sdf::~Sdf() {}

    const Formula& Sdf::getFormula() const
    {
        return formula;
    }

    void Sdf::setFormula(const Formula& newFormula)
    {
        formula = newFormula;
    }

    const std::string& Sdf::getExpression() const
    {
        return formula.getExpression();
    }

    void Sdf::setExpression(const std::string& expression)
    {
        formula.setExpression(expression);
    }

    const AABB3& Sdf::getAABB() const
    {
        return aabb;
    }

    void Sdf::setAABB(const AABB3& newAabb)
    {
        aabb = newAabb;
    }

    nlohmann::json Sdf::toJson() const
    {
        nlohmann::json json;
        json["Version"]   = 1;
        json["Formula"]   = formula.toJson();
        json["BoundsMin"] = { aabb.getMin().x, aabb.getMin().y, aabb.getMin().z };
        json["BoundsMax"] = { aabb.getMax().x, aabb.getMax().y, aabb.getMax().z };
        return json;
    }

    Sdf Sdf::fromJson(const nlohmann::json& json)
    {
        int version = json.at("Version").get<int>();
        if (version != 1)
        {
            throw std::runtime_error("Unsupported Sdf version");
        }

        Formula               parsedFormula = Formula::fromJson(json.at("Formula"));
        const nlohmann::json& minJson       = json.at("BoundsMin");
        const nlohmann::json& maxJson       = json.at("BoundsMax");
        AABB3                 parsedBounds(glm::dvec3(minJson.at(0).get<double>(), minJson.at(1).get<double>(), minJson.at(2).get<double>()),
                                           glm::dvec3(maxJson.at(0).get<double>(), maxJson.at(1).get<double>(), maxJson.at(2).get<double>()));
        return Sdf(parsedFormula, parsedBounds);
    }
}

#ifdef ISTESTPROJECT
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

TEST_CASE("Sdf/JsonRoundTrip")
{
    CADacomb::Sdf  sdf("output[0] = length(float3(input[0], input[1], input[2])) - 0.75;", CADacomb::AABB3(glm::dvec3(-2.0), glm::dvec3(2.0)));
    nlohmann::json json   = sdf.toJson();
    CADacomb::Sdf  loaded = CADacomb::Sdf::fromJson(json);

    CHECK(json.at("Version").get<int>() == 1);
    CHECK(loaded.getExpression() == sdf.getExpression());
    CHECK(loaded.getAABB().getMin().x == Catch::Approx(-2.0));
    CHECK(loaded.getAABB().getMax().z == Catch::Approx(2.0));
}
#endif

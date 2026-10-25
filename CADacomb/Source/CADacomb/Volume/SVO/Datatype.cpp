#include "Datatype.h"

namespace CADacomb
{
    std::string toString(Datatype datatype)
    {
        switch (datatype)
        {
            case Datatype::Bool:
                return "Bool";
            case Datatype::UInt8:
                return "UInt8";
            case Datatype::UInt16:
                return "UInt16";
            case Datatype::UInt32:
                return "UInt32";
            case Datatype::UInt64:
                return "UInt64";
            case Datatype::Int8:
                return "Int8";
            case Datatype::Int16:
                return "Int16";
            case Datatype::Int32:
                return "Int32";
            case Datatype::Int64:
                return "Int64";
            case Datatype::Float:
                return "Float";
            case Datatype::Double:
                return "Double";
            case Datatype::Vec3:
                return "Vec3";
            case Datatype::DVec3:
                return "DVec3";
            case Datatype::IVec3:
                return "IVec3";
            case Datatype::Custom:
                return "Custom";
        }
        return "Custom";
    }

    Datatype datatypeFromString(const std::string& str)
    {
        if (str == "Bool" || str == "bool")
        {
            return Datatype::Bool;
        }
        if (str == "UInt8" || str == "uint8" || str == "uint8_t")
        {
            return Datatype::UInt8;
        }
        if (str == "UInt16" || str == "uint16" || str == "uint16_t")
        {
            return Datatype::UInt16;
        }
        if (str == "UInt32" || str == "uint32" || str == "uint32_t")
        {
            return Datatype::UInt32;
        }
        if (str == "UInt64" || str == "uint64" || str == "uint64_t")
        {
            return Datatype::UInt64;
        }
        if (str == "Int8" || str == "int8" || str == "int8_t")
        {
            return Datatype::Int8;
        }
        if (str == "Int16" || str == "int16" || str == "int16_t")
        {
            return Datatype::Int16;
        }
        if (str == "Int32" || str == "int32" || str == "int32_t" || str == "int" || str == "Int")
        {
            return Datatype::Int32;
        }
        if (str == "Int64" || str == "int64" || str == "int64_t")
        {
            return Datatype::Int64;
        }
        if (str == "Float" || str == "float")
        {
            return Datatype::Float;
        }
        if (str == "Double" || str == "double")
        {
            return Datatype::Double;
        }
        if (str == "Vec3" || str == "vec3")
        {
            return Datatype::Vec3;
        }
        if (str == "DVec3" || str == "dvec3")
        {
            return Datatype::DVec3;
        }
        if (str == "IVec3" || str == "ivec3")
        {
            return Datatype::IVec3;
        }
        return Datatype::Custom;
    }
}

#ifdef ISTESTPROJECT
#include <catch2/catch_test_macros.hpp>

namespace CADacomb
{
    TEST_CASE("Datatype/ToStringAndFromString")
    {
        CHECK(toString(Datatype::Bool) == "Bool");
        CHECK(toString(Datatype::UInt8) == "UInt8");
        CHECK(toString(Datatype::UInt32) == "UInt32");
        CHECK(toString(Datatype::Int32) == "Int32");
        CHECK(toString(Datatype::Float) == "Float");
        CHECK(toString(Datatype::Double) == "Double");
        CHECK(toString(Datatype::Vec3) == "Vec3");
        CHECK(toString(Datatype::Custom) == "Custom");

        CHECK(datatypeFromString("Bool") == Datatype::Bool);
        CHECK(datatypeFromString("bool") == Datatype::Bool);
        CHECK(datatypeFromString("uint8_t") == Datatype::UInt8);
        CHECK(datatypeFromString("int") == Datatype::Int32);
        CHECK(datatypeFromString("float") == Datatype::Float);
        CHECK(datatypeFromString("double") == Datatype::Double);
        CHECK(datatypeFromString("Vec3") == Datatype::Vec3);
        CHECK(datatypeFromString("Unknown") == Datatype::Custom);
    }

    TEST_CASE("Datatype/TemplateHelper")
    {
        CHECK(getDatatype<bool>() == Datatype::Bool);
        CHECK(getDatatype<uint8_t>() == Datatype::UInt8);
        CHECK(getDatatype<uint16_t>() == Datatype::UInt16);
        CHECK(getDatatype<uint32_t>() == Datatype::UInt32);
        CHECK(getDatatype<uint64_t>() == Datatype::UInt64);
        CHECK(getDatatype<int8_t>() == Datatype::Int8);
        CHECK(getDatatype<int16_t>() == Datatype::Int16);
        CHECK(getDatatype<int32_t>() == Datatype::Int32);
        CHECK(getDatatype<int64_t>() == Datatype::Int64);
        CHECK(getDatatype<float>() == Datatype::Float);
        CHECK(getDatatype<double>() == Datatype::Double);
        CHECK(getDatatype<glm::vec3>() == Datatype::Vec3);
        CHECK(getDatatype<glm::dvec3>() == Datatype::DVec3);
        CHECK(getDatatype<glm::ivec3>() == Datatype::IVec3);
    }
}
#endif

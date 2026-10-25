#pragma once

#include <cstdint>
#include <glm/glm.hpp>
#include <string>
#include <type_traits>

namespace CADacomb
{
    enum class Datatype
    {
        Bool,
        UInt8,
        UInt16,
        UInt32,
        UInt64,
        Int8,
        Int16,
        Int32,
        Int64,
        Float,
        Double,
        Vec3,
        DVec3,
        IVec3,
        Custom
    };

    std::string toString(Datatype datatype);
    Datatype    datatypeFromString(const std::string& str);

    template<typename T>
    Datatype getDatatype();
}

namespace CADacomb
{
    template<typename T>
    Datatype getDatatype()
    {
        if constexpr (std::is_same_v<T, bool>)
        {
            return Datatype::Bool;
        }
        else if constexpr (std::is_same_v<T, uint8_t>)
        {
            return Datatype::UInt8;
        }
        else if constexpr (std::is_same_v<T, uint16_t>)
        {
            return Datatype::UInt16;
        }
        else if constexpr (std::is_same_v<T, uint32_t>)
        {
            return Datatype::UInt32;
        }
        else if constexpr (std::is_same_v<T, uint64_t>)
        {
            return Datatype::UInt64;
        }
        else if constexpr (std::is_same_v<T, int8_t>)
        {
            return Datatype::Int8;
        }
        else if constexpr (std::is_same_v<T, int16_t>)
        {
            return Datatype::Int16;
        }
        else if constexpr (std::is_same_v<T, int32_t>)
        {
            return Datatype::Int32;
        }
        else if constexpr (std::is_same_v<T, int64_t>)
        {
            return Datatype::Int64;
        }
        else if constexpr (std::is_same_v<T, float>)
        {
            return Datatype::Float;
        }
        else if constexpr (std::is_same_v<T, double>)
        {
            return Datatype::Double;
        }
        else if constexpr (std::is_same_v<T, glm::vec3>)
        {
            return Datatype::Vec3;
        }
        else if constexpr (std::is_same_v<T, glm::dvec3>)
        {
            return Datatype::DVec3;
        }
        else if constexpr (std::is_same_v<T, glm::ivec3>)
        {
            return Datatype::IVec3;
        }
        else
        {
            return Datatype::Custom;
        }
    }
}

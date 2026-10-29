#include "GUID.h"

#include <iomanip>
#include <random>
#include <sstream>

namespace CADacomb
{
    GUID::GUID()
      : value("")
    {
    }

    GUID::GUID(const std::string& value)
      : value(value)
    {
    }

    GUID::~GUID() {}

    GUID GUID::random()
    {
        static thread_local std::random_device  rd;
        static thread_local std::mt19937_64     gen(rd());
        std::uniform_int_distribution<uint64_t> dist;

        uint64_t part1 = dist(gen);
        uint64_t part2 = dist(gen);

        part1 = (part1 & 0xFFFFFFFFFFFF0FFFULL) | 0x0000000000004000ULL;
        part2 = (part2 & 0x3FFFFFFFFFFFFFFFULL) | 0x8000000000000000ULL;

        std::stringstream ss;
        ss << std::hex << std::setfill('0');
        ss << std::setw(8) << (part1 >> 32);
        ss << "-";
        ss << std::setw(4) << ((part1 >> 16) & 0xFFFFULL);
        ss << "-";
        ss << std::setw(4) << (part1 & 0xFFFFULL);
        ss << "-";
        ss << std::setw(4) << (part2 >> 48);
        ss << "-";
        ss << std::setw(12) << (part2 & 0xFFFFFFFFFFFFULL);

        return GUID(ss.str());
    }

    const std::string& GUID::toString() const
    {
        return value;
    }

    bool GUID::operator==(const GUID& other) const
    {
        return value == other.value;
    }

    bool GUID::operator!=(const GUID& other) const
    {
        return value != other.value;
    }

    bool GUID::operator<(const GUID& other) const
    {
        return value < other.value;
    }
}

#ifdef ISTESTPROJECT
#include <catch2/catch_test_macros.hpp>

TEST_CASE("GUID/DefaultAndStringConstruction")
{
    CADacomb::GUID empty;
    CHECK(empty.toString().empty());

    CADacomb::GUID custom("12345678-1234-4234-8234-123456789abc");
    CHECK(custom.toString() == "12345678-1234-4234-8234-123456789abc");
}

TEST_CASE("GUID/EqualityAndComparison")
{
    CADacomb::GUID g1("abc");
    CADacomb::GUID g2("abc");
    CADacomb::GUID g3("def");

    CHECK(g1 == g2);
    CHECK(g1 != g3);
    CHECK(g1 < g3);
}

TEST_CASE("GUID/RandomGeneration")
{
    CADacomb::GUID g1 = CADacomb::GUID::random();
    CADacomb::GUID g2 = CADacomb::GUID::random();

    CHECK(g1.toString().length() == 36);
    CHECK(g2.toString().length() == 36);
    CHECK(g1 != g2);
}
#endif

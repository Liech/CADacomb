#include "CADacomb/Util/Binary/BinaryReader.h"

namespace CADacomb
{
    BinaryReader::BinaryReader(std::istream& stream)
      : stream(stream)
    {
    }

    BinaryReader::~BinaryReader() {}
}

#ifdef ISTESTPROJECT
#include "CADacomb/Util/Binary/BinaryWriter.h"

#include <catch2/catch_test_macros.hpp>
#include <cstdint>
#include <glm/glm.hpp>
#include <sstream>

TEST_CASE("BinaryReader/RoundTripsScalars")
{
    std::stringstream      buffer;
    CADacomb::BinaryWriter writer(buffer);
    writer << uint64_t(123456789012345ull) << 3.5f << int16_t(-7);

    CADacomb::BinaryReader reader(buffer);
    uint64_t               a = 0;
    float                  b = 0.0f;
    int16_t                c = 0;
    reader >> a >> b >> c;

    CHECK(a == 123456789012345ull);
    CHECK(b == 3.5f);
    CHECK(c == -7);
}

TEST_CASE("BinaryReader/RoundTripsVectorsOfStructs")
{
    std::vector<glm::dvec3> original = { glm::dvec3(1.0, 2.0, 3.0), glm::dvec3(-4.0, 5.5, 6.25) };

    std::stringstream      buffer;
    CADacomb::BinaryWriter writer(buffer);
    writer << original;

    std::vector<glm::dvec3> restored(original.size());
    CADacomb::BinaryReader  reader(buffer);
    reader >> restored;

    CHECK(restored == original);
}

TEST_CASE("BinaryReader/ReadPastEndLeavesStreamFailed")
{
    std::stringstream      buffer;
    CADacomb::BinaryReader reader(buffer);
    uint32_t               value = 1;
    reader >> value;

    CHECK(!buffer);
    CHECK(value == 0);
}
#endif

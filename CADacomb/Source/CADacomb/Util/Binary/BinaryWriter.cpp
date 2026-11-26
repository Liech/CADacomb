#include "CADacomb/Util/Binary/BinaryWriter.h"

namespace CADacomb
{
    BinaryWriter::BinaryWriter(std::ostream& stream)
      : stream(stream)
    {
    }

    BinaryWriter::~BinaryWriter() {}
}

#ifdef ISTESTPROJECT
#include <catch2/catch_test_macros.hpp>
#include <cstdint>
#include <sstream>

TEST_CASE("BinaryWriter/WritesNativeByteLayout")
{
    std::ostringstream     out;
    CADacomb::BinaryWriter writer(out);
    uint32_t               value = 0x01020304u;
    writer << value;

    std::string bytes = out.str();
    REQUIRE(bytes.size() == sizeof(uint32_t));
    uint32_t firstByte = static_cast<unsigned char>(bytes[0]);
    CHECK((firstByte == 0x04u || firstByte == 0x01u));
}

TEST_CASE("BinaryWriter/WritesVectorWithoutLengthPrefix")
{
    std::ostringstream     out;
    CADacomb::BinaryWriter writer(out);
    writer << std::vector<uint16_t>{ 1, 2, 3 };
    CHECK(out.str().size() == 3 * sizeof(uint16_t));
}

TEST_CASE("BinaryWriter/ChainsMultipleValues")
{
    std::ostringstream     out;
    CADacomb::BinaryWriter writer(out);
    writer << uint8_t(1) << uint16_t(2) << uint32_t(3);
    CHECK(out.str().size() == 7);
}
#endif

#include "SVOFile.h"

#include <cctype>
#include <filesystem>
#include <sstream>

namespace CADacomb
{
    SVOFile::SVOFile() {}

    SVOFile::~SVOFile() {}

    void SVOFile::writeMagicAndHeader(std::ostream& stream, const nlohmann::json& header)
    {
        const std::string magic = "CADACOMBSVO";
        stream.write(magic.data(), static_cast<std::streamsize>(magic.size()));

        std::string dumped = header.dump() + "\n";
        stream.write(dumped.data(), static_cast<std::streamsize>(dumped.size()));
    }

    nlohmann::json SVOFile::parseHeader(std::istream& stream)
    {
        char magicBuffer[11];
        stream.read(magicBuffer, 11);
        if (stream.gcount() != 11 || std::string(magicBuffer, 11) != "CADACOMBSVO")
        {
            throw std::runtime_error("Invalid SVO magic bytes");
        }

        char ch = 0;
        while (stream.get(ch))
        {
            if (!std::isspace(static_cast<unsigned char>(ch)))
            {
                break;
            }
        }
        if (ch != '{')
        {
            throw std::runtime_error("Invalid SVO header: expected '{'");
        }

        std::string jsonStr;
        jsonStr.push_back(ch);
        int  braceDepth = 1;
        bool inString   = false;
        bool escape     = false;

        while (braceDepth > 0 && stream.get(ch))
        {
            jsonStr.push_back(ch);
            if (escape)
            {
                escape = false;
                continue;
            }
            if (ch == '\\' && inString)
            {
                escape = true;
                continue;
            }
            if (ch == '"')
            {
                inString = !inString;
                continue;
            }
            if (!inString)
            {
                if (ch == '{')
                {
                    braceDepth++;
                }
                else if (ch == '}')
                {
                    braceDepth--;
                }
            }
        }

        if (braceDepth != 0)
        {
            throw std::runtime_error("Incomplete SVO JSON header");
        }

        while (stream.peek() == '\r' || stream.peek() == '\n')
        {
            stream.get();
        }

        return nlohmann::json::parse(jsonStr);
    }

    nlohmann::json SVOFile::readHeader(const std::string& filePath)
    {
        std::ifstream stream(filePath, std::ios::binary);
        if (!stream.is_open())
        {
            throw std::runtime_error("Could not open file for reading: " + filePath);
        }
        return readHeader(stream);
    }

    nlohmann::json SVOFile::readHeader(std::istream& stream)
    {
        return parseHeader(stream);
    }

    std::unique_ptr<AnySVO> SVOFile::loadAny(const std::string& filePath, AABB3& outAABB, Datatype& outDatatype)
    {
        std::ifstream stream(filePath, std::ios::binary);
        if (!stream.is_open())
        {
            throw std::runtime_error("Could not open file for reading: " + filePath);
        }
        return loadAny(stream, outAABB, outDatatype);
    }

    std::unique_ptr<AnySVO> SVOFile::loadAny(std::istream& stream, AABB3& outAABB, Datatype& outDatatype)
    {
        nlohmann::json header = parseHeader(stream);
        outDatatype           = datatypeFromString(header.value("datatype", ""));

        switch (outDatatype)
        {
            case Datatype::Bool:
                return loadFromStreamWithHeader<bool>(stream, header, outAABB);
            case Datatype::UInt8:
                return loadFromStreamWithHeader<uint8_t>(stream, header, outAABB);
            case Datatype::UInt16:
                return loadFromStreamWithHeader<uint16_t>(stream, header, outAABB);
            case Datatype::UInt32:
                return loadFromStreamWithHeader<uint32_t>(stream, header, outAABB);
            case Datatype::UInt64:
                return loadFromStreamWithHeader<uint64_t>(stream, header, outAABB);
            case Datatype::Int8:
                return loadFromStreamWithHeader<int8_t>(stream, header, outAABB);
            case Datatype::Int16:
                return loadFromStreamWithHeader<int16_t>(stream, header, outAABB);
            case Datatype::Int32:
                return loadFromStreamWithHeader<int32_t>(stream, header, outAABB);
            case Datatype::Int64:
                return loadFromStreamWithHeader<int64_t>(stream, header, outAABB);
            case Datatype::Float:
                return loadFromStreamWithHeader<float>(stream, header, outAABB);
            case Datatype::Double:
                return loadFromStreamWithHeader<double>(stream, header, outAABB);
            case Datatype::Vec3:
                return loadFromStreamWithHeader<glm::vec3>(stream, header, outAABB);
            case Datatype::DVec3:
                return loadFromStreamWithHeader<glm::dvec3>(stream, header, outAABB);
            case Datatype::IVec3:
                return loadFromStreamWithHeader<glm::ivec3>(stream, header, outAABB);
            case Datatype::Custom:
                break;
        }

        throw std::runtime_error("Unsupported datatype for loadAny: " + header.value("datatype", ""));
    }

    void SVOFile::save(const std::string& filePath, const AnySVO& anySvo, const AABB3& aabb)
    {
        std::ofstream stream(filePath, std::ios::binary);
        if (!stream.is_open())
        {
            throw std::runtime_error("Could not open file for writing: " + filePath);
        }
        save(stream, anySvo, aabb);
    }

    void SVOFile::save(std::ostream& stream, const AnySVO& anySvo, const AABB3& aabb)
    {
        if (auto* b = dynamic_cast<const SVO<bool>*>(&anySvo))
        {
            save<bool>(stream, *b, aabb);
            return;
        }
        if (auto* u8 = dynamic_cast<const SVO<uint8_t>*>(&anySvo))
        {
            save<uint8_t>(stream, *u8, aabb);
            return;
        }
        if (auto* u16 = dynamic_cast<const SVO<uint16_t>*>(&anySvo))
        {
            save<uint16_t>(stream, *u16, aabb);
            return;
        }
        if (auto* u32 = dynamic_cast<const SVO<uint32_t>*>(&anySvo))
        {
            save<uint32_t>(stream, *u32, aabb);
            return;
        }
        if (auto* u64 = dynamic_cast<const SVO<uint64_t>*>(&anySvo))
        {
            save<uint64_t>(stream, *u64, aabb);
            return;
        }
        if (auto* i8 = dynamic_cast<const SVO<int8_t>*>(&anySvo))
        {
            save<int8_t>(stream, *i8, aabb);
            return;
        }
        if (auto* i16 = dynamic_cast<const SVO<int16_t>*>(&anySvo))
        {
            save<int16_t>(stream, *i16, aabb);
            return;
        }
        if (auto* i32 = dynamic_cast<const SVO<int32_t>*>(&anySvo))
        {
            save<int32_t>(stream, *i32, aabb);
            return;
        }
        if (auto* i64 = dynamic_cast<const SVO<int64_t>*>(&anySvo))
        {
            save<int64_t>(stream, *i64, aabb);
            return;
        }
        if (auto* f = dynamic_cast<const SVO<float>*>(&anySvo))
        {
            save<float>(stream, *f, aabb);
            return;
        }
        if (auto* d = dynamic_cast<const SVO<double>*>(&anySvo))
        {
            save<double>(stream, *d, aabb);
            return;
        }
        if (auto* v = dynamic_cast<const SVO<glm::vec3>*>(&anySvo))
        {
            save<glm::vec3>(stream, *v, aabb);
            return;
        }
        if (auto* dv = dynamic_cast<const SVO<glm::dvec3>*>(&anySvo))
        {
            save<glm::dvec3>(stream, *dv, aabb);
            return;
        }
        if (auto* iv = dynamic_cast<const SVO<glm::ivec3>*>(&anySvo))
        {
            save<glm::ivec3>(stream, *iv, aabb);
            return;
        }

        throw std::runtime_error("Unsupported AnySVO concrete type for save");
    }

    std::unique_ptr<AnySVO> SVOFile::load(const std::string& filePath, AABB3& outAABB)
    {
        Datatype detectedType = Datatype::Custom;
        return loadAny(filePath, outAABB, detectedType);
    }

    std::unique_ptr<AnySVO> SVOFile::load(std::istream& stream, AABB3& outAABB)
    {
        Datatype detectedType = Datatype::Custom;
        return loadAny(stream, outAABB, detectedType);
    }
}

#ifdef ISTESTPROJECT
#include <catch2/catch_test_macros.hpp>

namespace CADacomb
{
    TEST_CASE("SVOFile/SaveAndLoadFloat")
    {
        SVO<float> svo(16, 0.0f);
        svo.setVoxel(glm::ivec3(0, 0, 0), 1.5f);
        svo.setVoxel(glm::ivec3(5, 5, 5), 3.25f);
        svo.setVoxel(glm::ivec3(15, 15, 15), 9.0f);

        AABB3 originalBounds(glm::dvec3(-10.0, -20.0, -30.0), glm::dvec3(10.0, 20.0, 30.0));

        std::stringstream ss;
        SVOFile::save(ss, svo, originalBounds);

        AABB3 loadedBounds;
        auto  loadedSvo = SVOFile::load<float>(ss, loadedBounds);

        REQUIRE(loadedSvo != nullptr);
        CHECK(loadedBounds.getMin() == originalBounds.getMin());
        CHECK(loadedBounds.getMax() == originalBounds.getMax());
        CHECK(loadedSvo->getDimension() == 16);
        CHECK(loadedSvo->getVoxel(glm::ivec3(0, 0, 0)) == 1.5f);
        CHECK(loadedSvo->getVoxel(glm::ivec3(5, 5, 5)) == 3.25f);
        CHECK(loadedSvo->getVoxel(glm::ivec3(15, 15, 15)) == 9.0f);
        CHECK(loadedSvo->getVoxel(glm::ivec3(1, 1, 1)) == 0.0f);
        CHECK(loadedSvo->getRaw().size() == svo.getRaw().size());
        CHECK(loadedSvo->getParents().size() == svo.getParents().size());
        CHECK(loadedSvo->getFreeBlocks().size() == svo.getFreeBlocks().size());
    }

    TEST_CASE("SVOFile/SaveAndLoadBool")
    {
        SVO<bool> svo(8, false);
        svo.setVoxel(glm::ivec3(1, 2, 3), true);
        svo.setVoxel(glm::ivec3(7, 7, 7), true);

        AABB3 originalBounds(glm::dvec3(0.0), glm::dvec3(8.0));

        std::stringstream ss;
        SVOFile::save(ss, svo, originalBounds);

        AABB3 loadedBounds;
        auto  loadedSvo = SVOFile::load<bool>(ss, loadedBounds);

        REQUIRE(loadedSvo != nullptr);
        CHECK(loadedBounds.getMin() == originalBounds.getMin());
        CHECK(loadedBounds.getMax() == originalBounds.getMax());
        CHECK(loadedSvo->getDimension() == 8);
        CHECK(loadedSvo->getVoxel(glm::ivec3(1, 2, 3)) == true);
        CHECK(loadedSvo->getVoxel(glm::ivec3(7, 7, 7)) == true);
        CHECK(loadedSvo->getVoxel(glm::ivec3(0, 0, 0)) == false);
    }

    TEST_CASE("SVOFile/SaveAndLoadUInt8")
    {
        SVO<uint8_t> svo(4, static_cast<uint8_t>(42));
        svo.setVoxel(glm::ivec3(2, 2, 2), static_cast<uint8_t>(200));

        AABB3 originalBounds(glm::dvec3(1.0, 2.0, 3.0), glm::dvec3(4.0, 5.0, 6.0));

        std::stringstream ss;
        SVOFile::save(ss, svo, originalBounds);

        AABB3 loadedBounds;
        auto  loadedSvo = SVOFile::load<uint8_t>(ss, loadedBounds);

        REQUIRE(loadedSvo != nullptr);
        CHECK(loadedSvo->getVoxel(glm::ivec3(2, 2, 2)) == static_cast<uint8_t>(200));
        CHECK(loadedSvo->getVoxel(glm::ivec3(0, 0, 0)) == static_cast<uint8_t>(42));
    }

    TEST_CASE("SVOFile/LoadAny")
    {
        SVO<int32_t> svo(8, -1);
        svo.setVoxel(glm::ivec3(3, 3, 3), 999);

        AABB3 originalBounds(glm::dvec3(0.0), glm::dvec3(1.0));

        std::stringstream ss;
        SVOFile::save(ss, svo, originalBounds);

        AABB3    loadedBounds;
        Datatype detectedType = Datatype::Custom;
        auto     anyTree      = SVOFile::loadAny(ss, loadedBounds, detectedType);

        REQUIRE(anyTree != nullptr);
        CHECK(detectedType == Datatype::Int32);

        auto* typedTree = dynamic_cast<SVO<int32_t>*>(anyTree.get());
        REQUIRE(typedTree != nullptr);
        CHECK(typedTree->getVoxel(glm::ivec3(3, 3, 3)) == 999);
        CHECK(typedTree->getVoxel(glm::ivec3(0, 0, 0)) == -1);
    }

    TEST_CASE("SVOFile/InvalidMagicThrows")
    {
        std::stringstream ss;
        ss << "BADMAGICBYTE{\"version\":0}";

        AABB3 bounds;
        CHECK_THROWS_AS(SVOFile::load<float>(ss, bounds), std::runtime_error);
    }

    TEST_CASE("SVOFile/AnySVOSaveAndLoad")
    {
        SVO<bool> svo(16, false);
        svo.setVoxel(glm::ivec3(3, 4, 5), true);
        AABB3 originalBounds(glm::dvec3(-2.0), glm::dvec3(2.0));

        std::stringstream ss;
        const AnySVO&     anyRef = svo;
        SVOFile::save(ss, anyRef, originalBounds);

        AABB3                   loadedBounds;
        std::unique_ptr<AnySVO> loadedAny = SVOFile::load(ss, loadedBounds);
        REQUIRE(loadedAny != nullptr);
        CHECK(loadedBounds.getMin() == originalBounds.getMin());

        auto* typed = dynamic_cast<SVO<bool>*>(loadedAny.get());
        REQUIRE(typed != nullptr);
        CHECK(typed->getVoxel(glm::ivec3(3, 4, 5)) == true);
        CHECK(typed->getVoxel(glm::ivec3(0, 0, 0)) == false);
    }

    TEST_CASE("SVOFile/HeaderInspection")
    {
        SVO<float> svo(16, 1.0f);
        AABB3      bounds(glm::dvec3(0.0), glm::dvec3(16.0));

        std::stringstream ss;
        SVOFile::save(ss, svo, bounds);

        nlohmann::json header = SVOFile::readHeader(ss);
        CHECK(header["version"] == 0);
        CHECK(header["dimension"] == 16);
        CHECK(header["datatype"] == "Float");
        CHECK(header["mergeStrategy"] == "Mean");
    }

    TEST_CASE("SVOFile/FileIOMethods")
    {
        std::string tempPath = (std::filesystem::temp_directory_path() / "test_svo_file.cadacombsvo").string();

        SVO<float> svo(8, 2.5f);
        svo.setVoxel(glm::ivec3(2, 3, 4), 7.5f);
        AABB3 bounds(glm::dvec3(-5.0), glm::dvec3(5.0));

        SVOFile::save(tempPath, svo, bounds);

        nlohmann::json header = SVOFile::readHeader(tempPath);
        CHECK(header["dimension"] == 8);

        AABB3 loadedBounds;
        auto  loadedSvo = SVOFile::load<float>(tempPath, loadedBounds);
        REQUIRE(loadedSvo != nullptr);
        CHECK(loadedSvo->getVoxel(glm::ivec3(2, 3, 4)) == 7.5f);
        CHECK(loadedBounds.getMin() == bounds.getMin());

        std::filesystem::remove(tempPath);
    }
}
#endif

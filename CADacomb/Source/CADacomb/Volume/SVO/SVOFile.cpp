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

    void SVOFile::save(const std::string& filePath, const AnySVO& anySvo, const AABB3& aabb, const glm::dmat4& transformation)
    {
        std::ofstream stream(filePath, std::ios::binary);
        if (!stream.is_open())
        {
            throw std::runtime_error("Could not open file for writing: " + filePath);
        }
        save(stream, anySvo, aabb, transformation);
    }

    void SVOFile::save(std::ostream& stream, const AnySVO& anySvo, const AABB3& aabb, const glm::dmat4& transformation)
    {
        if (auto* b = dynamic_cast<const SVO<bool>*>(&anySvo))
        {
            save<bool>(stream, *b, aabb, transformation);
            return;
        }
        if (auto* u8 = dynamic_cast<const SVO<uint8_t>*>(&anySvo))
        {
            save<uint8_t>(stream, *u8, aabb, transformation);
            return;
        }
        if (auto* u16 = dynamic_cast<const SVO<uint16_t>*>(&anySvo))
        {
            save<uint16_t>(stream, *u16, aabb, transformation);
            return;
        }
        if (auto* u32 = dynamic_cast<const SVO<uint32_t>*>(&anySvo))
        {
            save<uint32_t>(stream, *u32, aabb, transformation);
            return;
        }
        if (auto* u64 = dynamic_cast<const SVO<uint64_t>*>(&anySvo))
        {
            save<uint64_t>(stream, *u64, aabb, transformation);
            return;
        }
        if (auto* i8 = dynamic_cast<const SVO<int8_t>*>(&anySvo))
        {
            save<int8_t>(stream, *i8, aabb, transformation);
            return;
        }
        if (auto* i16 = dynamic_cast<const SVO<int16_t>*>(&anySvo))
        {
            save<int16_t>(stream, *i16, aabb, transformation);
            return;
        }
        if (auto* i32 = dynamic_cast<const SVO<int32_t>*>(&anySvo))
        {
            save<int32_t>(stream, *i32, aabb, transformation);
            return;
        }
        if (auto* i64 = dynamic_cast<const SVO<int64_t>*>(&anySvo))
        {
            save<int64_t>(stream, *i64, aabb, transformation);
            return;
        }
        if (auto* f = dynamic_cast<const SVO<float>*>(&anySvo))
        {
            save<float>(stream, *f, aabb, transformation);
            return;
        }
        if (auto* d = dynamic_cast<const SVO<double>*>(&anySvo))
        {
            save<double>(stream, *d, aabb, transformation);
            return;
        }
        if (auto* v = dynamic_cast<const SVO<glm::vec3>*>(&anySvo))
        {
            save<glm::vec3>(stream, *v, aabb, transformation);
            return;
        }
        if (auto* dv = dynamic_cast<const SVO<glm::dvec3>*>(&anySvo))
        {
            save<glm::dvec3>(stream, *dv, aabb, transformation);
            return;
        }
        if (auto* iv = dynamic_cast<const SVO<glm::ivec3>*>(&anySvo))
        {
            save<glm::ivec3>(stream, *iv, aabb, transformation);
            return;
        }

        throw std::runtime_error("Unsupported AnySVO concrete type for save");
    }

    std::unique_ptr<AnySVO> SVOFile::load(const std::string& filePath, AABB3& outAABB, glm::dmat4& outTransformation)
    {
        std::ifstream stream(filePath, std::ios::binary);
        if (!stream.is_open())
        {
            throw std::runtime_error("Could not open file for reading: " + filePath);
        }
        return load(stream, outAABB, outTransformation);
    }

    std::unique_ptr<AnySVO> SVOFile::load(std::istream& stream, AABB3& outAABB, glm::dmat4& outTransformation)
    {
        nlohmann::json header = parseHeader(stream);
        return loadAny(stream, header, outAABB, outTransformation);
    }

    std::unique_ptr<AnySVO> SVOFile::loadAny(std::istream& stream, const nlohmann::json& header, AABB3& outAABB, glm::dmat4& outTransformation)
    {
        Datatype datatype = datatypeFromString(header.value("datatype", ""));

        switch (datatype)
        {
            case Datatype::Bool:
                return loadFromStreamWithHeader<bool>(stream, header, outAABB, outTransformation);
            case Datatype::UInt8:
                return loadFromStreamWithHeader<uint8_t>(stream, header, outAABB, outTransformation);
            case Datatype::UInt16:
                return loadFromStreamWithHeader<uint16_t>(stream, header, outAABB, outTransformation);
            case Datatype::UInt32:
                return loadFromStreamWithHeader<uint32_t>(stream, header, outAABB, outTransformation);
            case Datatype::UInt64:
                return loadFromStreamWithHeader<uint64_t>(stream, header, outAABB, outTransformation);
            case Datatype::Int8:
                return loadFromStreamWithHeader<int8_t>(stream, header, outAABB, outTransformation);
            case Datatype::Int16:
                return loadFromStreamWithHeader<int16_t>(stream, header, outAABB, outTransformation);
            case Datatype::Int32:
                return loadFromStreamWithHeader<int32_t>(stream, header, outAABB, outTransformation);
            case Datatype::Int64:
                return loadFromStreamWithHeader<int64_t>(stream, header, outAABB, outTransformation);
            case Datatype::Float:
                return loadFromStreamWithHeader<float>(stream, header, outAABB, outTransformation);
            case Datatype::Double:
                return loadFromStreamWithHeader<double>(stream, header, outAABB, outTransformation);
            case Datatype::Vec3:
                return loadFromStreamWithHeader<glm::vec3>(stream, header, outAABB, outTransformation);
            case Datatype::DVec3:
                return loadFromStreamWithHeader<glm::dvec3>(stream, header, outAABB, outTransformation);
            case Datatype::IVec3:
                return loadFromStreamWithHeader<glm::ivec3>(stream, header, outAABB, outTransformation);
            case Datatype::Custom:
                break;
        }

        throw std::runtime_error("Unsupported datatype for load: " + header.value("datatype", ""));
    }
}

#ifdef ISTESTPROJECT
#include <catch2/catch_test_macros.hpp>
#include <glm/gtc/matrix_transform.hpp>

namespace CADacomb
{
    TEST_CASE("SVOFile/SaveAndLoadFloat")
    {
        SVO<float> svo(16, 0.0f);
        svo.setVoxel(glm::ivec3(0, 0, 0), 1.5f);
        svo.setVoxel(glm::ivec3(5, 5, 5), 3.25f);
        svo.setVoxel(glm::ivec3(15, 15, 15), 9.0f);

        AABB3      originalBounds(glm::dvec3(-10.0, -20.0, -30.0), glm::dvec3(10.0, 20.0, 30.0));
        glm::dmat4 originalTransform = glm::translate(glm::dmat4(1.0), glm::dvec3(1.0, 2.0, 3.0));

        std::stringstream ss;
        SVOFile::save(ss, svo, originalBounds, originalTransform);

        AABB3      loadedBounds;
        glm::dmat4 loadedTransform;
        auto       loadedSvo = SVOFile::load<float>(ss, loadedBounds, loadedTransform);

        REQUIRE(loadedSvo != nullptr);
        CHECK(loadedBounds.getMin() == originalBounds.getMin());
        CHECK(loadedBounds.getMax() == originalBounds.getMax());
        CHECK(loadedTransform == originalTransform);
        CHECK(loadedSvo->getDimension() == 16);
        CHECK(loadedSvo->getVoxel(glm::ivec3(0, 0, 0)) == 1.5f);
        CHECK(loadedSvo->getVoxel(glm::ivec3(5, 5, 5)) == 3.25f);
        CHECK(loadedSvo->getVoxel(glm::ivec3(15, 15, 15)) == 9.0f);
        CHECK(loadedSvo->getVoxel(glm::ivec3(1, 1, 1)) == 0.0f);
        CHECK(loadedSvo->getRaw().size() == svo.getRaw().size());
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

        AABB3      loadedBounds;
        glm::dmat4 loadedTransform;
        auto       loadedSvo = SVOFile::load<bool>(ss, loadedBounds, loadedTransform);

        REQUIRE(loadedSvo != nullptr);
        CHECK(loadedBounds.getMin() == originalBounds.getMin());
        CHECK(loadedBounds.getMax() == originalBounds.getMax());
        CHECK(loadedTransform == glm::dmat4(1.0));
        CHECK(loadedSvo->getDimension() == 8);
        CHECK(loadedSvo->getVoxel(glm::ivec3(1, 2, 3)) == true);
        CHECK(loadedSvo->getVoxel(glm::ivec3(7, 7, 7)) == true);
        CHECK(loadedSvo->getVoxel(glm::ivec3(0, 0, 0)) == false);
    }

    TEST_CASE("SVOFile/SaveAndLoadUInt8")
    {
        SVO<uint8_t> svo(4, static_cast<uint8_t>(42));
        svo.setVoxel(glm::ivec3(2, 2, 2), static_cast<uint8_t>(200));

        AABB3      originalBounds(glm::dvec3(1.0, 2.0, 3.0), glm::dvec3(4.0, 5.0, 6.0));
        glm::dmat4 originalTransform = glm::scale(glm::dmat4(1.0), glm::dvec3(2.0, 3.0, 4.0));

        std::stringstream ss;
        SVOFile::save(ss, svo, originalBounds, originalTransform);

        AABB3      loadedBounds;
        glm::dmat4 loadedTransform;
        auto       loadedSvo = SVOFile::load<uint8_t>(ss, loadedBounds, loadedTransform);

        REQUIRE(loadedSvo != nullptr);
        CHECK(loadedBounds.getMin() == originalBounds.getMin());
        CHECK(loadedTransform == originalTransform);
        CHECK(loadedSvo->getVoxel(glm::ivec3(2, 2, 2)) == static_cast<uint8_t>(200));
        CHECK(loadedSvo->getVoxel(glm::ivec3(0, 0, 0)) == static_cast<uint8_t>(42));
    }

    TEST_CASE("SVOFile/InvalidMagicThrows")
    {
        std::stringstream ss;
        ss << "BADMAGICBYTE{\"version\":0}";

        AABB3      bounds;
        glm::dmat4 transform;
        CHECK_THROWS_AS(SVOFile::load<float>(ss, bounds, transform), std::runtime_error);
    }

    TEST_CASE("SVOFile/AnySVOSaveAndLoad")
    {
        SVO<bool> svo(16, false);
        svo.setVoxel(glm::ivec3(3, 4, 5), true);
        AABB3      originalBounds(glm::dvec3(-2.0), glm::dvec3(2.0));
        glm::dmat4 originalTransform = glm::translate(glm::dmat4(1.0), glm::dvec3(7.0, 8.0, 9.0));

        std::stringstream ss;
        const AnySVO&     anyRef = svo;
        SVOFile::save(ss, anyRef, originalBounds, originalTransform);

        AABB3                   loadedBounds;
        glm::dmat4              loadedTransform;
        std::unique_ptr<AnySVO> loadedAny = SVOFile::load(ss, loadedBounds, loadedTransform);
        REQUIRE(loadedAny != nullptr);
        CHECK(loadedBounds.getMin() == originalBounds.getMin());
        CHECK(loadedTransform == originalTransform);

        auto* typed = dynamic_cast<SVO<bool>*>(loadedAny.get());
        REQUIRE(typed != nullptr);
        CHECK(typed->getVoxel(glm::ivec3(3, 4, 5)) == true);
        CHECK(typed->getVoxel(glm::ivec3(0, 0, 0)) == false);
    }

    TEST_CASE("SVOFile/FileIOMethods")
    {
        std::string tempPath = (std::filesystem::temp_directory_path() / "test_svo_file.cadacombsvo").string();

        SVO<float> svo(8, 2.5f);
        svo.setVoxel(glm::ivec3(2, 3, 4), 7.5f);
        AABB3      bounds(glm::dvec3(-5.0), glm::dvec3(5.0));
        glm::dmat4 transform = glm::translate(glm::dmat4(1.0), glm::dvec3(0.5, 1.5, 2.5));

        SVOFile::save(tempPath, svo, bounds, transform);

        AABB3      loadedBounds;
        glm::dmat4 loadedTransform;
        auto       loadedSvo = SVOFile::load<float>(tempPath, loadedBounds, loadedTransform);
        REQUIRE(loadedSvo != nullptr);
        CHECK(loadedSvo->getVoxel(glm::ivec3(2, 3, 4)) == 7.5f);
        CHECK(loadedBounds.getMin() == bounds.getMin());
        CHECK(loadedTransform == transform);

        std::filesystem::remove(tempPath);
    }
}
#endif

#pragma once

#include <fstream>
#include <istream>
#include <memory>
#include <nlohmann/json.hpp>
#include <ostream>
#include <stdexcept>
#include <string>

#include "CADacomb/Util/AABB3.h"
#include "CADacomb/Util/ProgressBar.h"
#include "CADacomb/Volume/MergeStrategy/MergeStrategyType.h"
#include "CADacomb/Volume/SVO/AnySVO.h"
#include "CADacomb/Volume/SVO/Datatype.h"
#include "CADacomb/Volume/SVO/SVO.h"

namespace CADacomb
{
    // Loads and saves SVO octrees, bounding boxes, and transformations from and to the CADACOMBSVO file format.
    class SVOFile
    {
      public:
        SVOFile();
        virtual ~SVOFile();

        template<typename T>
        static void save(const std::string& filePath, const SVO<T>& svo, const AABB3& aabb, const glm::dmat4& transformation = glm::dmat4(1.0));
        template<typename T>
        static void save(std::ostream& stream, const SVO<T>& svo, const AABB3& aabb, const glm::dmat4& transformation = glm::dmat4(1.0));

        static void save(const std::string& filePath, const AnySVO& anySvo, const AABB3& aabb, const glm::dmat4& transformation = glm::dmat4(1.0));
        static void save(std::ostream& stream, const AnySVO& anySvo, const AABB3& aabb, const glm::dmat4& transformation = glm::dmat4(1.0));

        template<typename T>
        static std::unique_ptr<SVO<T>> load(const std::string& filePath, AABB3& outAABB, glm::dmat4& outTransformation);
        template<typename T>
        static std::unique_ptr<SVO<T>> load(std::istream& stream, AABB3& outAABB, glm::dmat4& outTransformation);

        static std::unique_ptr<AnySVO> load(const std::string& filePath, AABB3& outAABB, glm::dmat4& outTransformation);
        static std::unique_ptr<AnySVO> load(std::istream& stream, AABB3& outAABB, glm::dmat4& outTransformation);

      private:
        static void                    writeMagicAndHeader(std::ostream& stream, const nlohmann::json& header);
        static nlohmann::json          parseHeader(std::istream& stream);
        static std::unique_ptr<AnySVO> loadAny(std::istream& stream, const nlohmann::json& header, AABB3& outAABB, glm::dmat4& outTransformation);

        template<typename T>
        static std::unique_ptr<SVO<T>> loadFromStreamWithHeader(std::istream& stream, const nlohmann::json& header, AABB3& outAABB, glm::dmat4& outTransformation);
    };
}

namespace CADacomb
{
    template<typename T>
    void SVOFile::save(const std::string& filePath, const SVO<T>& svo, const AABB3& aabb, const glm::dmat4& transformation)
    {
        std::ofstream stream(filePath, std::ios::binary);
        if (!stream.is_open())
        {
            throw std::runtime_error("Could not open file for writing: " + filePath);
        }
        save(stream, svo, aabb, transformation);
    }

    template<typename T>
    void SVOFile::save(std::ostream& stream, const SVO<T>& svo, const AABB3& aabb, const glm::dmat4& transformation)
    {
        nlohmann::json header;
        header["version"] = 1;
        header["aabb"]    = {
            { "min", { aabb.getMin().x, aabb.getMin().y, aabb.getMin().z } },
            { "max", { aabb.getMax().x, aabb.getMax().y, aabb.getMax().z } }
        };
        header["dimension"]     = svo.getDimension();
        header["mergeStrategy"] = toString(getMergeStrategyType(svo.getMergeStrategy()));
        header["datatype"]      = toString(getDatatype<T>());

        nlohmann::json transformJson = nlohmann::json::array();
        for (int column = 0; column < 4; column++)
        {
            for (int row = 0; row < 4; row++)
            {
                transformJson.push_back(transformation[column][row]);
            }
        }
        header["transformation"] = transformJson;

        writeMagicAndHeader(stream, header);

        const auto& nodes     = svo.getRaw();
        uint64_t    nodeCount = static_cast<uint64_t>(nodes.size());
        stream.write(reinterpret_cast<const char*>(&nodeCount), sizeof(nodeCount));

        ProgressBar bar(nodes.size(), "Saving SVO nodes");
        for (size_t i = 0; i < nodes.size(); i++)
        {
            const auto& node         = nodes[i];
            uint32_t    childPointer = node.childPointer;
            stream.write(reinterpret_cast<const char*>(&childPointer), sizeof(childPointer));

            if constexpr (std::is_same_v<T, bool>)
            {
                uint8_t boolByte = node.value ? 1 : 0;
                stream.write(reinterpret_cast<const char*>(&boolByte), sizeof(boolByte));
            }
            else
            {
                stream.write(reinterpret_cast<const char*>(&node.value), sizeof(T));
            }

            bar.addProgress(1);
            if (bar.isCanceled())
            {
                throw std::runtime_error("Saving SVO canceled");
            }
        }

        uint64_t parentCount = 0;
        stream.write(reinterpret_cast<const char*>(&parentCount), sizeof(parentCount));

        const auto& freeBlocks     = svo.getFreeBlocks();
        uint64_t    freeBlockCount = static_cast<uint64_t>(freeBlocks.size());
        stream.write(reinterpret_cast<const char*>(&freeBlockCount), sizeof(freeBlockCount));
        if (freeBlockCount > 0)
        {
            stream.write(reinterpret_cast<const char*>(freeBlocks.data()), freeBlockCount * sizeof(uint32_t));
        }
    }

    template<typename T>
    std::unique_ptr<SVO<T>> SVOFile::load(const std::string& filePath, AABB3& outAABB, glm::dmat4& outTransformation)
    {
        std::ifstream stream(filePath, std::ios::binary);
        if (!stream.is_open())
        {
            throw std::runtime_error("Could not open file for reading: " + filePath);
        }
        return load<T>(stream, outAABB, outTransformation);
    }

    template<typename T>
    std::unique_ptr<SVO<T>> SVOFile::load(std::istream& stream, AABB3& outAABB, glm::dmat4& outTransformation)
    {
        nlohmann::json header = parseHeader(stream);
        return loadFromStreamWithHeader<T>(stream, header, outAABB, outTransformation);
    }

    template<typename T>
    std::unique_ptr<SVO<T>> SVOFile::loadFromStreamWithHeader(std::istream& stream, const nlohmann::json& header, AABB3& outAABB, glm::dmat4& outTransformation)
    {
        uint32_t    dimension        = header.value("dimension", 0u);
        std::string mergeStrategyStr = header.value("mergeStrategy", "");

        if (header.contains("aabb"))
        {
            const auto& aabbJson = header.at("aabb");
            if (aabbJson.contains("min") && aabbJson.at("min").is_array() && aabbJson.at("min").size() >= 3 && aabbJson.contains("max") && aabbJson.at("max").is_array() &&
                aabbJson.at("max").size() >= 3)
            {
                glm::dvec3 minPoint(aabbJson.at("min").at(0).get<double>(), aabbJson.at("min").at(1).get<double>(), aabbJson.at("min").at(2).get<double>());
                glm::dvec3 maxPoint(aabbJson.at("max").at(0).get<double>(), aabbJson.at("max").at(1).get<double>(), aabbJson.at("max").at(2).get<double>());
                outAABB = AABB3(minPoint, maxPoint);
            }
        }

        outTransformation = glm::dmat4(1.0);
        if (header.contains("transformation"))
        {
            const auto& transformJson = header.at("transformation");
            for (int column = 0; column < 4; column++)
            {
                for (int row = 0; row < 4; row++)
                {
                    outTransformation[column][row] = transformJson.at(column * 4 + row).get<double>();
                }
            }
        }

        uint64_t nodeCount = 0;
        stream.read(reinterpret_cast<char*>(&nodeCount), sizeof(nodeCount));
        if (!stream)
        {
            throw std::runtime_error("Failed to read node count");
        }

        ProgressBar             bar(nodeCount, "Loading SVO nodes");
        PagedVector<SVONode<T>> nodes(nodeCount);
        for (uint64_t i = 0; i < nodeCount; i++)
        {
            uint32_t childPointer = 0xFFFFFFFF;
            stream.read(reinterpret_cast<char*>(&childPointer), sizeof(childPointer));
            T value{};
            if constexpr (std::is_same_v<T, bool>)
            {
                uint8_t boolByte = 0;
                stream.read(reinterpret_cast<char*>(&boolByte), sizeof(boolByte));
                value = (boolByte != 0);
            }
            else
            {
                stream.read(reinterpret_cast<char*>(&value), sizeof(T));
            }
            nodes[i].childPointer = childPointer;
            nodes[i].value        = value;

            bar.addProgress(1);
            if (bar.isCanceled())
            {
                throw std::runtime_error("Loading SVO canceled");
            }
        }

        uint64_t parentCount = 0;
        stream.read(reinterpret_cast<char*>(&parentCount), sizeof(parentCount));
        if (!stream)
        {
            throw std::runtime_error("Failed to read parent count");
        }
        if (parentCount > 0)
        {
            stream.seekg(static_cast<std::streamoff>(parentCount * sizeof(uint32_t)), std::ios::cur);
        }

        uint64_t freeBlockCount = 0;
        stream.read(reinterpret_cast<char*>(&freeBlockCount), sizeof(freeBlockCount));
        if (!stream)
        {
            throw std::runtime_error("Failed to read free block count");
        }
        std::vector<uint32_t> freeBlocks(freeBlockCount);
        if (freeBlockCount > 0)
        {
            stream.read(reinterpret_cast<char*>(freeBlocks.data()), freeBlockCount * sizeof(uint32_t));
        }

        const MergeStrategy<T>& strategy = getMergeStrategyInstance<T>(mergeStrategyStr);
        return std::make_unique<SVO<T>>(dimension, std::move(nodes), std::move(freeBlocks), strategy);
    }
}

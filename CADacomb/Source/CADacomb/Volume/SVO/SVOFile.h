#pragma once

#include <fstream>
#include <istream>
#include <memory>
#include <nlohmann/json.hpp>
#include <ostream>
#include <stdexcept>
#include <string>

#include "CADacomb/Volume/AABB3.h"
#include "CADacomb/Volume/MergeStrategy/MergeStrategyType.h"
#include "CADacomb/Volume/SVO/AnySVO.h"
#include "CADacomb/Volume/SVO/Datatype.h"
#include "CADacomb/Volume/SVO/SVO.h"

namespace CADacomb
{
    // Loads and saves SVO octrees and bounding boxes from and to the CADACOMBSVO file format.
    class SVOFile
    {
      public:
        SVOFile();
        virtual ~SVOFile();

        template<typename T>
        static void save(const std::string& filePath, const SVO<T>& svo, const AABB3& aabb);

        template<typename T>
        static void save(std::ostream& stream, const SVO<T>& svo, const AABB3& aabb);

        static void save(const std::string& filePath, const AnySVO& anySvo, const AABB3& aabb);
        static void save(std::ostream& stream, const AnySVO& anySvo, const AABB3& aabb);

        template<typename T>
        static std::unique_ptr<SVO<T>> load(const std::string& filePath, AABB3& outAABB);

        template<typename T>
        static std::unique_ptr<SVO<T>> load(std::istream& stream, AABB3& outAABB);

        static std::unique_ptr<AnySVO> load(const std::string& filePath, AABB3& outAABB);
        static std::unique_ptr<AnySVO> load(std::istream& stream, AABB3& outAABB);

        static nlohmann::json readHeader(const std::string& filePath);
        static nlohmann::json readHeader(std::istream& stream);

        static std::unique_ptr<AnySVO> loadAny(const std::string& filePath, AABB3& outAABB, Datatype& outDatatype);
        static std::unique_ptr<AnySVO> loadAny(std::istream& stream, AABB3& outAABB, Datatype& outDatatype);

        template<typename T>
        static std::unique_ptr<SVO<T>> loadFromStreamWithHeader(std::istream& stream, const nlohmann::json& header, AABB3& outAABB);

      private:
        static void           writeMagicAndHeader(std::ostream& stream, const nlohmann::json& header);
        static nlohmann::json parseHeader(std::istream& stream);
    };
}

namespace CADacomb
{
    template<typename T>
    void SVOFile::save(const std::string& filePath, const SVO<T>& svo, const AABB3& aabb)
    {
        std::ofstream stream(filePath, std::ios::binary);
        if (!stream.is_open())
        {
            throw std::runtime_error("Could not open file for writing: " + filePath);
        }
        save(stream, svo, aabb);
    }

    template<typename T>
    void SVOFile::save(std::ostream& stream, const SVO<T>& svo, const AABB3& aabb)
    {
        nlohmann::json header;
        header["version"] = 0;
        header["aabb"]    = {
            { "min", { aabb.getMin().x, aabb.getMin().y, aabb.getMin().z } },
            { "max", { aabb.getMax().x, aabb.getMax().y, aabb.getMax().z } }
        };
        header["dimension"]     = svo.getDimension();
        header["mergeStrategy"] = toString(getMergeStrategyType(svo.getMergeStrategy()));
        header["datatype"]      = toString(getDatatype<T>());

        writeMagicAndHeader(stream, header);

        const auto& nodes     = svo.getRaw();
        uint64_t    nodeCount = static_cast<uint64_t>(nodes.size());
        stream.write(reinterpret_cast<const char*>(&nodeCount), sizeof(nodeCount));

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
        }

        const auto& parents     = svo.getParents();
        uint64_t    parentCount = static_cast<uint64_t>(parents.size());
        stream.write(reinterpret_cast<const char*>(&parentCount), sizeof(parentCount));
        if (parentCount > 0)
        {
            stream.write(reinterpret_cast<const char*>(parents.data()), parentCount * sizeof(uint32_t));
        }

        const auto& freeBlocks     = svo.getFreeBlocks();
        uint64_t    freeBlockCount = static_cast<uint64_t>(freeBlocks.size());
        stream.write(reinterpret_cast<const char*>(&freeBlockCount), sizeof(freeBlockCount));
        if (freeBlockCount > 0)
        {
            stream.write(reinterpret_cast<const char*>(freeBlocks.data()), freeBlockCount * sizeof(uint32_t));
        }
    }

    template<typename T>
    std::unique_ptr<SVO<T>> SVOFile::load(const std::string& filePath, AABB3& outAABB)
    {
        std::ifstream stream(filePath, std::ios::binary);
        if (!stream.is_open())
        {
            throw std::runtime_error("Could not open file for reading: " + filePath);
        }
        return load<T>(stream, outAABB);
    }

    template<typename T>
    std::unique_ptr<SVO<T>> SVOFile::load(std::istream& stream, AABB3& outAABB)
    {
        nlohmann::json header = parseHeader(stream);
        return loadFromStreamWithHeader<T>(stream, header, outAABB);
    }

    template<typename T>
    std::unique_ptr<SVO<T>> SVOFile::loadFromStreamWithHeader(std::istream& stream, const nlohmann::json& header, AABB3& outAABB)
    {
        uint32_t    dimension        = header.value("dimension", 0u);
        std::string mergeStrategyStr = header.value("mergeStrategy", "");

        if (header.contains("aabb"))
        {
            const auto& aabbJson = header["aabb"];
            if (aabbJson.contains("min") && aabbJson["min"].is_array() && aabbJson["min"].size() >= 3 && aabbJson.contains("max") && aabbJson["max"].is_array() && aabbJson["max"].size() >= 3)
            {
                glm::dvec3 minPoint(aabbJson["min"][0].get<double>(), aabbJson["min"][1].get<double>(), aabbJson["min"][2].get<double>());
                glm::dvec3 maxPoint(aabbJson["max"][0].get<double>(), aabbJson["max"][1].get<double>(), aabbJson["max"][2].get<double>());
                outAABB = AABB3(minPoint, maxPoint);
            }
        }

        uint64_t nodeCount = 0;
        stream.read(reinterpret_cast<char*>(&nodeCount), sizeof(nodeCount));
        if (!stream)
        {
            throw std::runtime_error("Failed to read node count");
        }

        std::vector<SVONode<T>> nodes(nodeCount);
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
        }

        uint64_t parentCount = 0;
        stream.read(reinterpret_cast<char*>(&parentCount), sizeof(parentCount));
        if (!stream)
        {
            throw std::runtime_error("Failed to read parent count");
        }
        std::vector<uint32_t> parents(parentCount);
        if (parentCount > 0)
        {
            stream.read(reinterpret_cast<char*>(parents.data()), parentCount * sizeof(uint32_t));
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
        return std::make_unique<SVO<T>>(dimension, std::move(nodes), std::move(parents), std::move(freeBlocks), strategy);
    }
}

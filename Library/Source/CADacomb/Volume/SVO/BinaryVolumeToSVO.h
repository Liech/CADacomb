#pragma once

#include "CADacomb/Volume/Flat/BinaryVolume.h"
#include "CADacomb/Volume/MergeStrategy/MeanMergeStrategy.h"
#include "CADacomb/Volume/MergeStrategy/MostPrevalentMergeStrategy.h"
#include "SVO.h"
#include "SVOResult.h"

#include <algorithm>
#include <cassert>
#include <cstdint>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <memory>

namespace Library::Volume
{
    class BinaryVolumeToSVO
    {
      public:
        BinaryVolumeToSVO();
        virtual ~BinaryVolumeToSVO();

        template<typename T = bool>
        static std::unique_ptr<SVO<T>> convert(const BinaryVolume&     volume,
                                               T                       solidValue = static_cast<T>(true),
                                               T                       emptyValue = static_cast<T>(false),
                                               const MergeStrategy<T>& strategy   = SVO<T>::defaultMergeStrategy());

        template<typename T = bool>
        static std::unique_ptr<SVO<T>> convert(const BinaryVolume&     volume,
                                               uint32_t                targetDimension,
                                               const glm::ivec3&       offset     = glm::ivec3(0),
                                               T                       solidValue = static_cast<T>(true),
                                               T                       emptyValue = static_cast<T>(false),
                                               const MergeStrategy<T>& strategy   = SVO<T>::defaultMergeStrategy());

        template<typename T = bool>
        static SVOResult convertWithResult(const BinaryVolume&     volume,
                                           T                       solidValue = static_cast<T>(true),
                                           T                       emptyValue = static_cast<T>(false),
                                           const MergeStrategy<T>& strategy   = SVO<T>::defaultMergeStrategy());

        static glm::dmat4 getTransformation(const BinaryVolume& volume);
    };

    template<typename T>
    std::unique_ptr<SVO<T>> BinaryVolumeToSVO::convert(const BinaryVolume& volume, T solidValue, T emptyValue, const MergeStrategy<T>& strategy)
    {
        uint32_t maxDim = std::max({ static_cast<uint32_t>(volume.dimension.x), static_cast<uint32_t>(volume.dimension.y), static_cast<uint32_t>(volume.dimension.z) });
        if (maxDim == 0)
        {
            return nullptr;
        }

        uint32_t targetDimension = 1;
        while (targetDimension < maxDim)
        {
            targetDimension <<= 1;
        }

        return convert<T>(volume, targetDimension, glm::ivec3(0), solidValue, emptyValue, strategy);
    }

    template<typename T>
    std::unique_ptr<SVO<T>> BinaryVolumeToSVO::convert(const BinaryVolume& volume, uint32_t targetDimension, const glm::ivec3& offset, T solidValue, T emptyValue, const MergeStrategy<T>& strategy)
    {
        assert(targetDimension > 0 && (targetDimension & (targetDimension - 1)) == 0);

        auto svo = std::make_unique<SVO<T>>(targetDimension, emptyValue, strategy);

        const size_t dimX = static_cast<size_t>(volume.dimension.x);
        const size_t dimY = static_cast<size_t>(volume.dimension.y);
        const size_t dimZ = static_cast<size_t>(volume.dimension.z);

        for (size_t z = 0; z < dimZ; ++z)
        {
            for (size_t y = 0; y < dimY; ++y)
            {
                for (size_t x = 0; x < dimX; ++x)
                {
                    size_t idx = x + y * dimX + z * dimX * dimY;
                    if (idx < volume.data.size() && volume.data[idx])
                    {
                        glm::ivec3 targetPos = glm::ivec3(static_cast<int>(x), static_cast<int>(y), static_cast<int>(z)) + offset;
                        if (targetPos.x >= 0 && targetPos.x < static_cast<int>(targetDimension) && targetPos.y >= 0 && targetPos.y < static_cast<int>(targetDimension) && targetPos.z >= 0 &&
                            targetPos.z < static_cast<int>(targetDimension))
                        {
                            svo->setVoxel(targetPos, solidValue);
                        }
                    }
                }
            }
        }

        return svo;
    }

    template<typename T>
    SVOResult BinaryVolumeToSVO::convertWithResult(const BinaryVolume& volume, T solidValue, T emptyValue, const MergeStrategy<T>& strategy)
    {
        SVOResult res;
        res.tree           = convert<T>(volume, solidValue, emptyValue, strategy);
        res.transformation = getTransformation(volume);
        return res;
    }
}

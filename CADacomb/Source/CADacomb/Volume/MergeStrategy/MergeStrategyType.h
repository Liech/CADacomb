#pragma once

#include <string>
#include <type_traits>

namespace CADacomb
{
    template<typename T>
    class MergeStrategy;

    enum class MergeStrategyType
    {
        Mean,
        MostPrevalent,
        MostPrevalentExcept
    };

    std::string       toString(MergeStrategyType strategyType);
    MergeStrategyType mergeStrategyTypeFromString(const std::string& str);

    template<typename T>
    MergeStrategyType getMergeStrategyType(const MergeStrategy<T>& strategy);

    template<typename T>
    const MergeStrategy<T>& getMergeStrategyInstance(MergeStrategyType type);

    template<typename T>
    const MergeStrategy<T>& getMergeStrategyInstance(const std::string& strategyName);
}

#include "CADacomb/Volume/MergeStrategy/MeanMergeStrategy.h"
#include "CADacomb/Volume/MergeStrategy/MostPrevalentMergeStrategy.h"

namespace CADacomb
{
    template<typename T>
    MergeStrategyType getMergeStrategyType(const MergeStrategy<T>& strategy)
    {
        if constexpr (!std::is_same_v<T, bool> && requires(T a) { a + a; })
        {
            if (dynamic_cast<const MeanMergeStrategy<T>*>(&strategy) != nullptr)
            {
                return MergeStrategyType::Mean;
            }
        }
        if constexpr (requires(T a, T b) { a < b; } && requires(T a, T b) { a == b; })
        {
            if (dynamic_cast<const MostPrevalentMergeStrategy<T>*>(&strategy) != nullptr)
            {
                return MergeStrategyType::MostPrevalent;
            }
        }
        return MergeStrategyType::MostPrevalent;
    }

    template<typename T>
    const MergeStrategy<T>& getMergeStrategyInstance(MergeStrategyType type)
    {
        if (type == MergeStrategyType::Mean)
        {
            if constexpr (!std::is_same_v<T, bool> && requires(T a) { a + a; })
            {
                return MeanMergeStrategy<T>::instance();
            }
        }
        else if (type == MergeStrategyType::MostPrevalent)
        {
            if constexpr (requires(T a, T b) { a < b; } && requires(T a, T b) { a == b; })
            {
                return MostPrevalentMergeStrategy<T>::instance();
            }
        }

        if constexpr (std::is_same_v<T, bool>)
        {
            return MostPrevalentMergeStrategy<T>::instance();
        }
        else if constexpr (requires(T a) { a + a; })
        {
            return MeanMergeStrategy<T>::instance();
        }
        else if constexpr (requires(T a, T b) { a < b; } && requires(T a, T b) { a == b; })
        {
            return MostPrevalentMergeStrategy<T>::instance();
        }
    }

    template<typename T>
    const MergeStrategy<T>& getMergeStrategyInstance(const std::string& strategyName)
    {
        MergeStrategyType type = mergeStrategyTypeFromString(strategyName);
        return getMergeStrategyInstance<T>(type);
    }
}

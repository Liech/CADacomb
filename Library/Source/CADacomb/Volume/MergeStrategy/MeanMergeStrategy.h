#pragma once

#include "MergeStrategy.h"
#include <array>

namespace Library::Volume
{
    template<typename T>
    class MeanMergeStrategy : public MergeStrategy<T>
    {
      public:
        MeanMergeStrategy();
        virtual ~MeanMergeStrategy();

        T merge(const std::array<T, 8>& inputs) const override;

        static const MeanMergeStrategy<T>& instance();
    };

    template<typename T>
    MeanMergeStrategy<T>::MeanMergeStrategy()
    {
    }

    template<typename T>
    MeanMergeStrategy<T>::~MeanMergeStrategy()
    {
    }

    template<typename T>
    const MeanMergeStrategy<T>& MeanMergeStrategy<T>::instance()
    {
        static const MeanMergeStrategy<T> inst;
        return inst;
    }

    template<typename T>
    T MeanMergeStrategy<T>::merge(const std::array<T, 8>& inputs) const
    {
        T sum = inputs[0];
        for (size_t i = 1; i < 8; i++)
        {
            sum += inputs[i];
        }
        if constexpr (requires { typename T::value_type; })
        {
            return sum / static_cast<typename T::value_type>(8);
        }
        else
        {
            return sum / static_cast<T>(8);
        }
    }
}

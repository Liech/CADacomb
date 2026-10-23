#pragma once

#include "MergeStrategy.h"
#include <array>

namespace Library::Volume
{
    template<typename T, T ValueNotCounted>
    class MostPrevalentExceptMergeStrategy : public MergeStrategy<T>
    {
      public:
        MostPrevalentExceptMergeStrategy();
        virtual ~MostPrevalentExceptMergeStrategy();

        T merge(const std::array<T, 8>& inputs) const override;

        static const MostPrevalentExceptMergeStrategy<T, ValueNotCounted>& instance();
    };

    template<typename T, T ValueNotCounted>
    MostPrevalentExceptMergeStrategy<T, ValueNotCounted>::MostPrevalentExceptMergeStrategy()
    {
    }

    template<typename T, T ValueNotCounted>
    MostPrevalentExceptMergeStrategy<T, ValueNotCounted>::~MostPrevalentExceptMergeStrategy()
    {
    }

    template<typename T, T ValueNotCounted>
    const MostPrevalentExceptMergeStrategy<T, ValueNotCounted>& MostPrevalentExceptMergeStrategy<T, ValueNotCounted>::instance()
    {
        static const MostPrevalentExceptMergeStrategy<T, ValueNotCounted> inst;
        return inst;
    }

    template<typename T, T ValueNotCounted>
    T MostPrevalentExceptMergeStrategy<T, ValueNotCounted>::merge(const std::array<T, 8>& inputs) const
    {
        static_assert(requires(T a, T b) { a < b; } && requires(T a, T b) { a == b; });
        T   mostPrevalent = inputs[0];
        int maxCount      = 0;
        for (size_t i = 0; i < 8; i++)
        {
            int count = 0;
            for (size_t j = 0; j < 8; j++)
            {
                if (inputs[i] == inputs[j] && ValueNotCounted != inputs[i])
                {
                    count++;
                }
            }
            if (count > maxCount)
            {
                maxCount      = count;
                mostPrevalent = inputs[i];
            }
            else if (count == maxCount)
            {
                if (inputs[i] < mostPrevalent)
                {
                    mostPrevalent = inputs[i];
                }
            }
        }
        return mostPrevalent;
    }
}

#pragma once

#include "MergeStrategy.h"
#include <array>

namespace Library::Volume
{
    template<typename T>
    class MostPrevalentMergeStrategy : public MergeStrategy<T>
    {
      public:
        MostPrevalentMergeStrategy();
        virtual ~MostPrevalentMergeStrategy();

        T merge(const std::array<T, 8>& inputs) const override;

        static const MostPrevalentMergeStrategy<T>& instance();
    };

    template<typename T>
    MostPrevalentMergeStrategy<T>::MostPrevalentMergeStrategy()
    {
    }

    template<typename T>
    MostPrevalentMergeStrategy<T>::~MostPrevalentMergeStrategy()
    {
    }

    template<typename T>
    const MostPrevalentMergeStrategy<T>& MostPrevalentMergeStrategy<T>::instance()
    {
        static const MostPrevalentMergeStrategy<T> inst;
        return inst;
    }

    template<typename T>
    T MostPrevalentMergeStrategy<T>::merge(const std::array<T, 8>& inputs) const
    {
        static_assert(requires(T a, T b) { a < b; } && requires(T a, T b) { a == b; });
        T   mostPrevalent = inputs[0];
        int maxCount      = 0;
        for (size_t i = 0; i < 8; i++)
        {
            int count = 0;
            for (size_t j = 0; j < 8; j++)
            {
                if (inputs[i] == inputs[j])
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

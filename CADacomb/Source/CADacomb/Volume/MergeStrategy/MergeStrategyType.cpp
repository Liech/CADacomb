#include "MergeStrategyType.h"

namespace CADacomb
{
    std::string toString(MergeStrategyType strategyType)
    {
        switch (strategyType)
        {
            case MergeStrategyType::Mean:
                return "Mean";
            case MergeStrategyType::MostPrevalent:
                return "MostPrevalent";
            case MergeStrategyType::MostPrevalentExcept:
                return "MostPrevalentExcept";
        }
        return "MostPrevalent";
    }

    MergeStrategyType mergeStrategyTypeFromString(const std::string& str)
    {
        if (str == "Mean" || str == "MeanMergeStrategy")
        {
            return MergeStrategyType::Mean;
        }
        if (str == "MostPrevalentExcept" || str == "MostPrevalentExceptMergeStrategy")
        {
            return MergeStrategyType::MostPrevalentExcept;
        }
        return MergeStrategyType::MostPrevalent;
    }
}

#ifdef ISTESTPROJECT
#include <catch2/catch_test_macros.hpp>

namespace CADacomb
{
    TEST_CASE("MergeStrategyType/ToStringAndFromString")
    {
        CHECK(toString(MergeStrategyType::Mean) == "Mean");
        CHECK(toString(MergeStrategyType::MostPrevalent) == "MostPrevalent");
        CHECK(toString(MergeStrategyType::MostPrevalentExcept) == "MostPrevalentExcept");

        CHECK(mergeStrategyTypeFromString("Mean") == MergeStrategyType::Mean);
        CHECK(mergeStrategyTypeFromString("MeanMergeStrategy") == MergeStrategyType::Mean);
        CHECK(mergeStrategyTypeFromString("MostPrevalent") == MergeStrategyType::MostPrevalent);
        CHECK(mergeStrategyTypeFromString("MostPrevalentMergeStrategy") == MergeStrategyType::MostPrevalent);
        CHECK(mergeStrategyTypeFromString("MostPrevalentExcept") == MergeStrategyType::MostPrevalentExcept);
        CHECK(mergeStrategyTypeFromString("Unknown") == MergeStrategyType::MostPrevalent);
    }

    TEST_CASE("MergeStrategyType/TypeDetectionAndInstance")
    {
        const auto& mean = MeanMergeStrategy<float>::instance();
        CHECK(getMergeStrategyType(mean) == MergeStrategyType::Mean);

        const auto& mostPrev = MostPrevalentMergeStrategy<float>::instance();
        CHECK(getMergeStrategyType(mostPrev) == MergeStrategyType::MostPrevalent);

        const auto& resolvedMean = getMergeStrategyInstance<float>(MergeStrategyType::Mean);
        CHECK(&resolvedMean == &mean);

        const auto& resolvedFromName = getMergeStrategyInstance<float>("Mean");
        CHECK(&resolvedFromName == &mean);
    }
}
#endif

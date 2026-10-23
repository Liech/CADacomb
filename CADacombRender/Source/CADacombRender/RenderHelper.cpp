#include "RenderHelper.h"

namespace CADacomb::Render
{
    RenderHelper::RenderHelper() {}

    RenderHelper::~RenderHelper() {}

    std::string RenderHelper::name() const
    {
        return "CADacombRender";
    }
}

#ifdef ISTESTPROJECT
#include <catch2/catch_test_macros.hpp>

TEST_CASE("RenderHelper/Initialization")
{
    CADacomb::Render::RenderHelper helper;
    REQUIRE(helper.name() == "CADacombRender");
}
#endif

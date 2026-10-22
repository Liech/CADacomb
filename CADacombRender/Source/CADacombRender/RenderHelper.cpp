#include "RenderHelper.h"

RenderHelper::RenderHelper() {}

RenderHelper::~RenderHelper() {}

std::string RenderHelper::name() const
{
    return "CADacombRender";
}

#ifdef ISTESTPROJECT
#include <catch2/catch_test_macros.hpp>

TEST_CASE("RenderHelper/Initialization")
{
    RenderHelper helper;
    REQUIRE(helper.name() == "CADacombRender");
}
#endif

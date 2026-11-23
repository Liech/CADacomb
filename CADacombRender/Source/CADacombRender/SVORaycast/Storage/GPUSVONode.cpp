#include "CADacombRender/SVORaycast/Storage/GPUSVONode.h"

namespace CADacomb::Render
{
    GPUSVONode::GPUSVONode() {}

    GPUSVONode::GPUSVONode(uint32_t childPointerInput, uint32_t materialIdInput, uint32_t validMaskInput)
      : childPointer(childPointerInput)
      , payload((materialIdInput & 0x00FFFFFFu) | ((validMaskInput & 0xFFu) << 24u))
    {
    }

    uint32_t GPUSVONode::getMaterialId() const
    {
        return payload & 0x00FFFFFFu;
    }

    uint32_t GPUSVONode::getValidMask() const
    {
        return payload >> 24u;
    }
}

#ifdef ISTESTPROJECT
#include <catch2/catch_test_macros.hpp>

namespace CADacomb::Render
{
    TEST_CASE("GPUSVONode/PayloadPacking")
    {
        GPUSVONode node(42u, 0x00123456u, 0xA5u);
        REQUIRE(node.childPointer == 42u);
        REQUIRE(node.getMaterialId() == 0x00123456u);
        REQUIRE(node.getValidMask() == 0xA5u);
    }
}
#endif

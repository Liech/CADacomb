#include "CADacombRender/SVORaycast/Storage/GPUSVONode.h"

namespace CADacomb::Render
{
    GPUSVONode::GPUSVONode() {}

    GPUSVONode::GPUSVONode(uint32_t childPointerInput, uint32_t valueInput)
      : childPointer(childPointerInput)
      , value(valueInput)
    {
    }
}

#pragma once

namespace CADacomb
{
    template<typename T, unsigned short brickSize = 8, bool padded = false>
    struct Brick
    {
      public:
        T data[(brickSize + (padded ? 2 : 0)) * (brickSize + (padded ? 2 : 0)) * (brickSize + (padded ? 2 : 0))];
    };
}

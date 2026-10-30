#pragma once

namespace MagicaVoxImporter
{
    struct Rotation
    {
        Rotation();
        virtual ~Rotation();

        int v00 = 0, v10 = 0, v20 = 0;
        int v01 = 0, v11 = 0, v21 = 0;
        int v02 = 0, v12 = 0, v22 = 0;
    };
}

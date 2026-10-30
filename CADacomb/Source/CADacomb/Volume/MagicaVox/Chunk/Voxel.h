#pragma once

namespace MagicaVoxImporter
{
    struct Voxel
    {
        Voxel();
        virtual ~Voxel();

        unsigned char x;
        unsigned char y;
        unsigned char z;
        unsigned char value;
    };
}

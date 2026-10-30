#pragma once

namespace MagicaVoxImporter
{
    struct Color
    {
        Color();
        virtual ~Color();

        unsigned char r;
        unsigned char g;
        unsigned char b;
        unsigned char a;
    };
}

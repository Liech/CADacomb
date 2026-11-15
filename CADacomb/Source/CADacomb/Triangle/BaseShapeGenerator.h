#pragma once

#include "CADacomb/Triangle/Triangulation.h"

namespace CADacomb
{
    class BaseShapeGenerator
    {
      public:
        static Triangulation cube();
        static Triangulation box(float width, float depth, float height);
        static Triangulation pyramid();
        static Triangulation cylinder(size_t segments, float height, float radius);
        static Triangulation cubeWithHole(size_t segments, float radius = 0.3f);
        static Triangulation cone(size_t segments, float radius, float height);
        static Triangulation frustum(size_t segments, float bottomRadius, float topRadius, float height);
        static Triangulation sphere(size_t segments, size_t rings, float radius);
        static Triangulation torus(size_t majorSegments, size_t minorSegments, float majorRadius, float minorRadius);
        static Triangulation capsule(size_t segments, size_t rings, float radius, float height);
        static Triangulation prism(size_t sides, float radius, float height);
        static Triangulation octahedron(float radius = 1.0f);
        static Triangulation lBracket(float length, float width, float height, float thickness);
        static Triangulation steppedBlock(float length, float width, float height);
        static Triangulation plateWithHoles(size_t segments, float length, float width, float thickness, float holeRadius);
    };
}

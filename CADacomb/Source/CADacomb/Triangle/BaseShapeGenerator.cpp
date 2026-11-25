#include "BaseShapeGenerator.h"

namespace CADacomb
{
    Triangulation BaseShapeGenerator::cube()
    {
        Triangulation m;
        m.vertices = {
            { 0, 0, 0 },
            { 1, 0, 0 },
            { 1, 1, 0 },
            { 0, 1, 0 },
            { 0, 0, 1 },
            { 1, 0, 1 },
            { 1, 1, 1 },
            { 0, 1, 1 }
        };
        m.indices = { 0, 3, 2, 0, 2, 1, 4, 5, 6, 4, 6, 7, 0, 1, 5, 0, 5, 4, 1, 2, 6, 1, 6, 5, 2, 3, 7, 2, 7, 6, 3, 0, 4, 3, 4, 7 };
        return m;
    }

    Triangulation BaseShapeGenerator::pyramid()
    {
        Triangulation m;
        m.vertices = {
            {   0,   0, 0 },
            {   1,   0, 0 },
            {   1,   1, 0 },
            {   0,   1, 0 }, // Base (0-3)
            { 0.5, 0.5, 1 }  // Apex (4)
        };
        m.indices = {
            0, 2, 1, 0, 3, 2, // Base triangles
            0, 1, 4,          // Side 1
            1, 2, 4,          // Side 2
            2, 3, 4,          // Side 3
            3, 0, 4           // Side 4
        };
        return m;
    }

    Triangulation BaseShapeGenerator::cylinder(size_t segments, float height, float radius)
    {
        Triangulation m;

        // 1. Generate Ring Vertices
        for (int i = 0; i < segments; ++i)
        {
            float theta = 2.0f * 3.1415926f * float(i) / float(segments);
            float x     = radius * cosf(theta);
            float y     = radius * sinf(theta);
            m.vertices.push_back({ x, y, 0 });      // Bottom ring
            m.vertices.push_back({ x, y, height }); // Top ring
        }

        // 2. Add Cap Center Vertices (Hubs)
        size_t bottomCenterIdx = m.vertices.size();
        m.vertices.push_back({ 0, 0, 0 }); // Center of bottom cap
        size_t topCenterIdx = m.vertices.size();
        m.vertices.push_back({ 0, 0, height }); // Center of top cap

        for (int i = 0; i < segments; ++i)
        {
            size_t b0 = i * 2;
            size_t t0 = i * 2 + 1;
            size_t b1 = ((i + 1) % segments) * 2;
            size_t t1 = ((i + 1) % segments) * 2 + 1;

            // Side Faces (The "Sleeve")
            m.indices.insert(m.indices.end(), { b0, b1, t1, b0, t1, t0 });

            // Bottom Cap (Triangle Fan)
            m.indices.insert(m.indices.end(), { bottomCenterIdx, b1, b0 });

            // Top Cap (Triangle Fan)
            m.indices.insert(m.indices.end(), { topCenterIdx, t0, t1 });
        }
        return m;
    }
    Triangulation BaseShapeGenerator::cubeWithHole(size_t segments, float radius)
    {
        Triangulation m;
        if (segments % 4 != 0)
            segments = (segments / 4 + 1) * 4;
        if (radius > 0.45f)
            radius = 0.45f;

        // 1. Outer Vertices
        m.vertices = {
            { 0, 0, 0 },
            { 1, 0, 0 },
            { 1, 1, 0 },
            { 0, 1, 0 }, // 0-3 Bottom (Z=0)
            { 0, 0, 1 },
            { 1, 0, 1 },
            { 1, 1, 1 },
            { 0, 1, 1 }  // 4-7 Top (Z=1)
        };

        // 2. Inner Vertices
        size_t innerStart = m.vertices.size();
        float  startAngle = 3.1415926f * 1.00f; // Aligning with corner 0
        for (size_t i = 0; i < segments; ++i)
        {
            float theta = startAngle + (2.0f * 3.1415926f * (float)i / (float)segments);
            float x     = 0.5f + radius * cosf(theta);
            float y     = 0.5f + radius * sinf(theta);
            m.vertices.push_back({ x, y, 0 }); // Bottom Inner (innerStart + i*2)
            m.vertices.push_back({ x, y, 1 }); // Top Inner (innerStart + i*2 + 1)
        }
        // 3. Outer Walls (Facing OUT)
        m.indices.insert(m.indices.end(),
                         {
                           0, 1, 5, 0, 5, 4, // Front
                           1, 2, 6, 1, 6, 5, // Right
                           2, 3, 7, 2, 7, 6, // Back
                           3, 0, 4, 3, 4, 7  // Left
                         });

        // 4. Inner Bore (Facing INSIDE the hole - outward from the solid)
        for (size_t i = 0; i < segments; ++i)
        {
            uint32_t b0 = (uint32_t)(innerStart + i * 2);
            uint32_t t0 = b0 + 1;
            uint32_t b1 = (uint32_t)(innerStart + ((i + 1) % segments) * 2);
            uint32_t t1 = b1 + 1;

            // To point normals toward the center of the hole:
            // b0 -> b1 -> t1 then b0 -> t1 -> t0
            m.indices.insert(m.indices.end(), { b0, t1, b1, t1, b0, t0 });
        }

        // 5. Caps (Top and Bottom)
        for (size_t i = 0; i < segments; ++i)
        {
            uint32_t b_in0 = (uint32_t)(innerStart + i * 2);
            uint32_t b_in1 = (uint32_t)(innerStart + ((i + 1) % segments) * 2);
            uint32_t t_in0 = b_in0 + 1;
            uint32_t t_in1 = b_in1 + 1;

            uint32_t corner = (uint32_t)((i * 4) / segments);
            uint32_t b_out  = corner;
            uint32_t t_out  = corner + 4;

            // TOP CAP (Z+) - CCW looking down: Outer -> Inner_Next -> Inner_Curr
            m.indices.insert(m.indices.end(), { t_out, t_in1, t_in0 });

            // BOTTOM CAP (Z-) - CCW looking up: Outer -> Inner_Curr -> Inner_Next
            m.indices.insert(m.indices.end(), { b_out, b_in0, b_in1 });

            // BRIDGE (Filling the square corners)
            if (((i + 1) * 4) / segments > corner)
            {
                uint32_t b_out_next = (uint32_t)((corner + 1) % 4);
                uint32_t t_out_next = b_out_next + 4;

                // TOP BRIDGE: Outer -> OuterNext -> InnerNext
                m.indices.insert(m.indices.end(), { t_out, t_out_next, t_in1 });
                // BOTTOM BRIDGE: Outer -> InnerNext -> OuterNext
                m.indices.insert(m.indices.end(), { b_out, b_in1, b_out_next });
            }
        }

        return m;
    }
    Triangulation BaseShapeGenerator::cone(size_t segments, float radius, float height)
    {
        Triangulation m;
        if (segments < 3)
            segments = 3; // Minimum for a volume

        // 1. Vertices: Bottom Ring
        for (size_t i = 0; i < segments; ++i)
        {
            float theta = 2.0f * 3.1415926f * (float)i / (float)segments;
            m.vertices.push_back({ radius * cosf(theta), radius * sinf(theta), 0 });
        }

        // 2. Apex and Base Center
        size_t apexIdx = m.vertices.size();
        m.vertices.push_back({ 0, 0, height }); // The tip

        size_t centerIdx = m.vertices.size();
        m.vertices.push_back({ 0, 0, 0 }); // Center of the bottom cap

        // 3. Indices
        for (size_t i = 0; i < segments; ++i)
        {
            uint32_t curr = (uint32_t)i;
            uint32_t next = (uint32_t)((i + 1) % segments);

            // Sides: Apex -> Next -> Curr (CCW facing out)
            m.indices.insert(m.indices.end(), { (uint32_t)apexIdx, curr, next });

            // Base Cap: Center -> Curr -> Next (CCW facing down)
            m.indices.insert(m.indices.end(), { (uint32_t)centerIdx, next, curr });
        }

        return m;
    }

    Triangulation BaseShapeGenerator::box(float width, float depth, float height)
    {
        float w = width > 0.0f ? width : 1.0f;
        float d = depth > 0.0f ? depth : 1.0f;
        float h = height > 0.0f ? height : 1.0f;

        Triangulation m;
        m.vertices = {
            { 0, 0, 0 },
            { w, 0, 0 },
            { w, d, 0 },
            { 0, d, 0 },
            { 0, 0, h },
            { w, 0, h },
            { w, d, h },
            { 0, d, h }
        };
        m.indices = { 0, 3, 2, 0, 2, 1, 4, 5, 6, 4, 6, 7, 0, 1, 5, 0, 5, 4, 1, 2, 6, 1, 6, 5, 2, 3, 7, 2, 7, 6, 3, 0, 4, 3, 4, 7 };
        return m;
    }

    Triangulation BaseShapeGenerator::frustum(size_t segments, float bottomRadius, float topRadius, float height)
    {
        if (segments < 3)
            segments = 3;
        float rBot = bottomRadius > 0.0f ? bottomRadius : 1.0f;
        float rTop = topRadius > 0.0f ? topRadius : 0.5f;
        float h    = height > 0.0f ? height : 1.0f;

        Triangulation m;
        float         twoPi = 6.28318530718f;
        for (size_t i = 0; i < segments; i++)
        {
            float theta = twoPi * static_cast<float>(i) / static_cast<float>(segments);
            float c     = cosf(theta);
            float s     = sinf(theta);
            m.vertices.push_back({ rBot * c, rBot * s, 0.0f });
            m.vertices.push_back({ rTop * c, rTop * s, h });
        }

        size_t bottomCenterIdx = m.vertices.size();
        m.vertices.push_back({ 0.0f, 0.0f, 0.0f });
        size_t topCenterIdx = m.vertices.size();
        m.vertices.push_back({ 0.0f, 0.0f, h });

        for (size_t i = 0; i < segments; i++)
        {
            size_t b0 = i * 2;
            size_t t0 = i * 2 + 1;
            size_t b1 = ((i + 1) % segments) * 2;
            size_t t1 = ((i + 1) % segments) * 2 + 1;

            m.indices.insert(m.indices.end(), { b0, b1, t1, b0, t1, t0 });
            m.indices.insert(m.indices.end(), { bottomCenterIdx, b1, b0 });
            m.indices.insert(m.indices.end(), { topCenterIdx, t0, t1 });
        }

        return m;
    }

    Triangulation BaseShapeGenerator::sphere(size_t segments, size_t rings, float radius)
    {
        if (segments < 3)
            segments = 3;
        if (rings < 2)
            rings = 2;
        float r = radius > 0.0f ? radius : 1.0f;

        Triangulation m;
        size_t        bottomPoleIdx = 0;
        m.vertices.push_back({ 0.0f, 0.0f, -r });

        float pi    = 3.14159265359f;
        float twoPi = 6.28318530718f;
        for (size_t ring = 1; ring < rings; ring++)
        {
            float phi    = -0.5f * pi + pi * static_cast<float>(ring) / static_cast<float>(rings);
            float cosPhi = cosf(phi);
            float sinPhi = sinf(phi);
            for (size_t seg = 0; seg < segments; seg++)
            {
                float theta = twoPi * static_cast<float>(seg) / static_cast<float>(segments);
                m.vertices.push_back({ r * cosPhi * cosf(theta), r * cosPhi * sinf(theta), r * sinPhi });
            }
        }

        size_t topPoleIdx = m.vertices.size();
        m.vertices.push_back({ 0.0f, 0.0f, r });

        for (size_t seg = 0; seg < segments; seg++)
        {
            size_t curr = 1 + seg;
            size_t next = 1 + ((seg + 1) % segments);
            m.indices.insert(m.indices.end(), { bottomPoleIdx, next, curr });
        }

        for (size_t ring = 0; ring + 2 < rings; ring++)
        {
            size_t row0 = 1 + ring * segments;
            size_t row1 = 1 + (ring + 1) * segments;
            for (size_t seg = 0; seg < segments; seg++)
            {
                size_t nextSeg = (seg + 1) % segments;
                size_t a       = row0 + seg;
                size_t b       = row0 + nextSeg;
                size_t c       = row1 + nextSeg;
                size_t d       = row1 + seg;
                m.indices.insert(m.indices.end(), { a, b, c, a, c, d });
            }
        }

        size_t lastRow = 1 + (rings - 2) * segments;
        for (size_t seg = 0; seg < segments; seg++)
        {
            size_t curr = lastRow + seg;
            size_t next = lastRow + ((seg + 1) % segments);
            m.indices.insert(m.indices.end(), { topPoleIdx, curr, next });
        }

        return m;
    }

    Triangulation BaseShapeGenerator::torus(size_t majorSegments, size_t minorSegments, float majorRadius, float minorRadius)
    {
        if (majorSegments < 3)
            majorSegments = 3;
        if (minorSegments < 3)
            minorSegments = 3;
        float rMinor = minorRadius > 0.0f ? minorRadius : 0.25f;
        float rMajor = majorRadius > rMinor ? majorRadius : rMinor + 0.5f;

        Triangulation m;
        float         twoPi = 6.28318530718f;
        for (size_t i = 0; i < majorSegments; i++)
        {
            float u    = twoPi * static_cast<float>(i) / static_cast<float>(majorSegments);
            float cosU = cosf(u);
            float sinU = sinf(u);
            for (size_t j = 0; j < minorSegments; j++)
            {
                float v      = twoPi * static_cast<float>(j) / static_cast<float>(minorSegments);
                float cosV   = cosf(v);
                float sinV   = sinf(v);
                float radial = rMajor + rMinor * cosV;
                m.vertices.push_back({ radial * cosU, radial * sinU, rMinor * sinV });
            }
        }

        for (size_t i = 0; i < majorSegments; i++)
        {
            size_t nextI = (i + 1) % majorSegments;
            for (size_t j = 0; j < minorSegments; j++)
            {
                size_t nextJ = (j + 1) % minorSegments;
                size_t a     = i * minorSegments + j;
                size_t b     = nextI * minorSegments + j;
                size_t c     = nextI * minorSegments + nextJ;
                size_t d     = i * minorSegments + nextJ;
                m.indices.insert(m.indices.end(), { a, b, c, a, c, d });
            }
        }

        return m;
    }

    Triangulation BaseShapeGenerator::capsule(size_t segments, size_t rings, float radius, float height)
    {
        if (segments < 3)
            segments = 3;
        if (rings < 2)
            rings = 2;
        float r = radius > 0.0f ? radius : 0.5f;
        float h = height >= 0.0f ? height : 1.0f;

        Triangulation m;
        size_t        bottomPoleIdx = 0;
        m.vertices.push_back({ 0.0f, 0.0f, -r });

        float halfPi = 1.57079632679f;
        float twoPi  = 6.28318530718f;

        for (size_t ring = 1; ring <= rings; ring++)
        {
            float phi    = -halfPi + halfPi * static_cast<float>(ring) / static_cast<float>(rings);
            float cosPhi = cosf(phi);
            float sinPhi = sinf(phi);
            for (size_t seg = 0; seg < segments; seg++)
            {
                float theta = twoPi * static_cast<float>(seg) / static_cast<float>(segments);
                m.vertices.push_back({ r * cosPhi * cosf(theta), r * cosPhi * sinf(theta), r * sinPhi });
            }
        }

        for (size_t ring = 0; ring < rings; ring++)
        {
            float phi    = halfPi * static_cast<float>(ring) / static_cast<float>(rings);
            float cosPhi = cosf(phi);
            float sinPhi = sinf(phi);
            for (size_t seg = 0; seg < segments; seg++)
            {
                float theta = twoPi * static_cast<float>(seg) / static_cast<float>(segments);
                m.vertices.push_back({ r * cosPhi * cosf(theta), r * cosPhi * sinf(theta), h + r * sinPhi });
            }
        }

        size_t topPoleIdx = m.vertices.size();
        m.vertices.push_back({ 0.0f, 0.0f, h + r });

        for (size_t seg = 0; seg < segments; seg++)
        {
            size_t curr = 1 + seg;
            size_t next = 1 + ((seg + 1) % segments);
            m.indices.insert(m.indices.end(), { bottomPoleIdx, next, curr });
        }

        size_t totalRings = 2 * rings;
        for (size_t ring = 0; ring + 1 < totalRings; ring++)
        {
            size_t row0 = 1 + ring * segments;
            size_t row1 = 1 + (ring + 1) * segments;
            for (size_t seg = 0; seg < segments; seg++)
            {
                size_t nextSeg = (seg + 1) % segments;
                size_t a       = row0 + seg;
                size_t b       = row0 + nextSeg;
                size_t c       = row1 + nextSeg;
                size_t d       = row1 + seg;
                m.indices.insert(m.indices.end(), { a, b, c, a, c, d });
            }
        }

        size_t lastRow = 1 + (totalRings - 1) * segments;
        for (size_t seg = 0; seg < segments; seg++)
        {
            size_t curr = lastRow + seg;
            size_t next = lastRow + ((seg + 1) % segments);
            m.indices.insert(m.indices.end(), { topPoleIdx, curr, next });
        }

        return m;
    }

    Triangulation BaseShapeGenerator::prism(size_t sides, float radius, float height)
    {
        if (sides < 3)
            sides = 3;
        return cylinder(sides, height > 0.0f ? height : 1.0f, radius > 0.0f ? radius : 1.0f);
    }

    Triangulation BaseShapeGenerator::octahedron(float radius)
    {
        float r = radius > 0.0f ? radius : 1.0f;

        Triangulation m;
        m.vertices = {
            {    r, 0.0f, 0.0f },
            {   -r, 0.0f, 0.0f },
            { 0.0f,    r, 0.0f },
            { 0.0f,   -r, 0.0f },
            { 0.0f, 0.0f,    r },
            { 0.0f, 0.0f,   -r }
        };
        m.indices = {
            0, 2, 4,
            2, 1, 4,
            1, 3, 4,
            3, 0, 4,
            2, 0, 5,
            1, 2, 5,
            3, 1, 5,
            0, 3, 5
        };
        return m;
    }

    Triangulation BaseShapeGenerator::lBracket(float length, float width, float height, float thickness)
    {
        float len = length > 0.2f ? length : 2.0f;
        float wid = width > 0.1f ? width : 1.0f;
        float hgt = height > 0.2f ? height : 2.0f;
        float t   = thickness > 0.05f ? thickness : 0.4f;
        if (t >= len)
            t = len * 0.4f;
        if (t >= hgt)
            t = hgt * 0.4f;

        Triangulation m;
        m.vertices = {
            { 0.0f, 0.0f, 0.0f },
            {  len, 0.0f, 0.0f },
            {  len, 0.0f,    t },
            {    t, 0.0f,    t },
            {    t, 0.0f,  hgt },
            { 0.0f, 0.0f,  hgt },
            { 0.0f,  wid, 0.0f },
            {  len,  wid, 0.0f },
            {  len,  wid,    t },
            {    t,  wid,    t },
            {    t,  wid,  hgt },
            { 0.0f,  wid,  hgt }
        };

        m.indices = {
            0, 1, 2, 0, 2, 3, 0, 3, 4, 0, 4, 5,
            6, 8, 7, 6, 9, 8, 6, 10, 9, 6, 11, 10
        };

        for (size_t i = 0; i < 6; i++)
        {
            size_t next = (i + 1) % 6;
            size_t b0   = i;
            size_t b1   = next;
            size_t t0   = 6 + i;
            size_t t1   = 6 + next;
            m.indices.insert(m.indices.end(), { b0, t0, t1, b0, t1, b1 });
        }

        return m;
    }

    Triangulation BaseShapeGenerator::steppedBlock(float length, float width, float height)
    {
        float len = length > 0.2f ? length : 2.0f;
        float wid = width > 0.1f ? width : 1.0f;
        float hgt = height > 0.2f ? height : 1.0f;
        float hx  = len * 0.5f;
        float hz  = hgt * 0.5f;

        Triangulation m;
        m.vertices = {
            { 0.0f, 0.0f, 0.0f },
            {  len, 0.0f, 0.0f },
            {  len, 0.0f,   hz },
            {   hx, 0.0f,   hz },
            {   hx, 0.0f,  hgt },
            { 0.0f, 0.0f,  hgt },
            { 0.0f,  wid, 0.0f },
            {  len,  wid, 0.0f },
            {  len,  wid,   hz },
            {   hx,  wid,   hz },
            {   hx,  wid,  hgt },
            { 0.0f,  wid,  hgt }
        };

        m.indices = {
            0, 1, 2, 0, 2, 3, 0, 3, 4, 0, 4, 5,
            6, 8, 7, 6, 9, 8, 6, 10, 9, 6, 11, 10
        };

        for (size_t i = 0; i < 6; i++)
        {
            size_t next = (i + 1) % 6;
            size_t b0   = i;
            size_t b1   = next;
            size_t t0   = 6 + i;
            size_t t1   = 6 + next;
            m.indices.insert(m.indices.end(), { b0, t0, t1, b0, t1, b1 });
        }

        return m;
    }

    Triangulation BaseShapeGenerator::plateWithHoles(size_t segments, float length, float width, float thickness, float holeRadius)
    {
        if (segments % 4 != 0 || segments == 0)
            segments = (segments / 4 + 1) * 4;

        float len = length > 0.2f ? length : 2.0f;
        float wid = width > 0.1f ? width : 1.0f;
        float thk = thickness > 0.05f ? thickness : 0.25f;
        float hx  = len * 0.5f;
        float maxR = 0.42f * (hx < wid ? hx : wid);
        float r    = holeRadius > 0.01f ? (holeRadius < maxR ? holeRadius : maxR) : maxR * 0.6f;

        Triangulation m;
        // Outer perimeter: 0=(0,0), 1=(hx,0), 2=(len,0), 3=(len,wid), 4=(hx,wid), 5=(0,wid)
        // Bottom (0..5), Top (6..11)
        m.vertices = {
            { 0.0f, 0.0f, 0.0f },
            {   hx, 0.0f, 0.0f },
            {  len, 0.0f, 0.0f },
            {  len,  wid, 0.0f },
            {   hx,  wid, 0.0f },
            { 0.0f,  wid, 0.0f },
            { 0.0f, 0.0f,  thk },
            {   hx, 0.0f,  thk },
            {  len, 0.0f,  thk },
            {  len,  wid,  thk },
            {   hx,  wid,  thk },
            { 0.0f,  wid,  thk }
        };

        for (size_t i = 0; i < 6; i++)
        {
            size_t next = (i + 1) % 6;
            m.indices.insert(m.indices.end(), { i, next, 6 + next, i, 6 + next, 6 + i });
        }

        float pi    = 3.14159265359f;
        float twoPi = 6.28318530718f;

        size_t leftCorners[4]  = { 0, 1, 4, 5 };
        size_t rightCorners[4] = { 1, 2, 3, 4 };

        for (int holeIdx = 0; holeIdx < 2; holeIdx++)
        {
            float   cx         = holeIdx == 0 ? hx * 0.5f : hx * 1.5f;
            float   cy         = wid * 0.5f;
            size_t* corners    = holeIdx == 0 ? leftCorners : rightCorners;
            size_t  innerStart = m.vertices.size();

            for (size_t i = 0; i < segments; i++)
            {
                float theta = pi + twoPi * static_cast<float>(i) / static_cast<float>(segments);
                float x     = cx + r * cosf(theta);
                float y     = cy + r * sinf(theta);
                m.vertices.push_back({ x, y, 0.0f });
                m.vertices.push_back({ x, y, thk });
            }

            for (size_t i = 0; i < segments; i++)
            {
                size_t b0 = innerStart + i * 2;
                size_t t0 = b0 + 1;
                size_t b1 = innerStart + ((i + 1) % segments) * 2;
                size_t t1 = b1 + 1;
                m.indices.insert(m.indices.end(), { b0, t1, b1, t1, b0, t0 });
            }

            for (size_t i = 0; i < segments; i++)
            {
                size_t bIn0 = innerStart + i * 2;
                size_t bIn1 = innerStart + ((i + 1) % segments) * 2;
                size_t tIn0 = bIn0 + 1;
                size_t tIn1 = bIn1 + 1;

                size_t cornerIdx = (i * 4) / segments;
                size_t bOut      = corners[cornerIdx];
                size_t tOut      = bOut + 6;

                m.indices.insert(m.indices.end(), { tOut, tIn1, tIn0 });
                m.indices.insert(m.indices.end(), { bOut, bIn0, bIn1 });

                if (((i + 1) * 4) / segments > cornerIdx)
                {
                    size_t bOutNext = corners[(cornerIdx + 1) % 4];
                    size_t tOutNext = bOutNext + 6;
                    m.indices.insert(m.indices.end(), { tOut, tOutNext, tIn1 });
                    m.indices.insert(m.indices.end(), { bOut, bIn1, bOutNext });
                }
            }
        }

        return m;
    }
}

#ifdef ISTESTPROJECT
#include "CADacomb/CAD/Conversion/ClusterToCadConverter.h"
#include "CADacomb/CAD/Conversion/NaiveMeshToCadConverter.h"
#include "CADacomb/CAD/Interface/Shape.h"
#include "CADacomb/CAD/Sm/KernelSm.h"
#include "CADacomb/Triangle/Clustering.h"
#include "CADacomb/Triangle/HalfEdge/HalfEdgeHealth.h"
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

TEST_CASE("BaseShapeGenerator/unitcube")
{
    auto tri = CADacomb::BaseShapeGenerator::cube();
    REQUIRE(CADacomb::HalfEdgeHealth::isHealthy(tri));
    REQUIRE(tri.vertices.size() == 8);
    REQUIRE(tri.indices.size() == 36);
}

TEST_CASE("BaseShapeGenerator/box")
{
    auto tri = CADacomb::BaseShapeGenerator::box(2.0f, 3.0f, 4.0f);
    REQUIRE(CADacomb::HalfEdgeHealth::isHealthy(tri));
    auto aabb = tri.getAABB();
    REQUIRE((aabb.first.x + aabb.second.x) == Catch::Approx(2.0));
    REQUIRE((aabb.first.y + aabb.second.y) == Catch::Approx(3.0));
    REQUIRE((aabb.first.z + aabb.second.z) == Catch::Approx(4.0));
}

TEST_CASE("BaseShapeGenerator/pyramid")
{
    auto tri = CADacomb::BaseShapeGenerator::pyramid();
    REQUIRE(CADacomb::HalfEdgeHealth::isHealthy(tri));
    REQUIRE(tri.vertices.size() == 5);
    REQUIRE(tri.indices.size() == 18);
}

TEST_CASE("BaseShapeGenerator/cylinder")
{
    auto tri = CADacomb::BaseShapeGenerator::cylinder(5, 1, 1);
    REQUIRE(CADacomb::HalfEdgeHealth::isHealthy(tri));
}

TEST_CASE("BaseShapeGenerator/cylinderHighRes")
{
    auto tri = CADacomb::BaseShapeGenerator::cylinder(32, 2.5f, 0.75f);
    REQUIRE(CADacomb::HalfEdgeHealth::isHealthy(tri));
}

TEST_CASE("BaseShapeGenerator/cubeWithHole")
{
    auto tri = CADacomb::BaseShapeGenerator::cubeWithHole(5);
    REQUIRE(CADacomb::HalfEdgeHealth::isHealthy(tri));
}

TEST_CASE("BaseShapeGenerator/cubeWithHoleHighRes")
{
    auto tri = CADacomb::BaseShapeGenerator::cubeWithHole(32, 0.25f);
    REQUIRE(CADacomb::HalfEdgeHealth::isHealthy(tri));
}

TEST_CASE("BaseShapeGenerator/cone")
{
    auto tri = CADacomb::BaseShapeGenerator::cone(10, 1, 1);
    REQUIRE(CADacomb::HalfEdgeHealth::isHealthy(tri));
}

TEST_CASE("BaseShapeGenerator/coneMinimal")
{
    auto tri = CADacomb::BaseShapeGenerator::cone(2, 2.0f, 3.0f);
    REQUIRE(CADacomb::HalfEdgeHealth::isHealthy(tri));
}

TEST_CASE("BaseShapeGenerator/frustum")
{
    auto tri = CADacomb::BaseShapeGenerator::frustum(16, 1.5f, 0.6f, 2.0f);
    REQUIRE(CADacomb::HalfEdgeHealth::isHealthy(tri));
    auto aabb = tri.getAABB();
    REQUIRE(aabb.first.z == Catch::Approx(0.0));
    REQUIRE((aabb.first.z + aabb.second.z) == Catch::Approx(2.0));
}

TEST_CASE("BaseShapeGenerator/frustumMinimal")
{
    auto tri = CADacomb::BaseShapeGenerator::frustum(3, 1.0f, 0.5f, 1.0f);
    REQUIRE(CADacomb::HalfEdgeHealth::isHealthy(tri));
}

TEST_CASE("BaseShapeGenerator/sphere")
{
    auto tri = CADacomb::BaseShapeGenerator::sphere(16, 8, 1.5f);
    REQUIRE(CADacomb::HalfEdgeHealth::isHealthy(tri));
    auto aabb = tri.getAABB();
    REQUIRE(aabb.first.z == Catch::Approx(-1.5));
    REQUIRE((aabb.first.z + aabb.second.z) == Catch::Approx(1.5));
}

TEST_CASE("BaseShapeGenerator/sphereMinimal")
{
    auto tri = CADacomb::BaseShapeGenerator::sphere(3, 2, 1.0f);
    REQUIRE(CADacomb::HalfEdgeHealth::isHealthy(tri));
}

TEST_CASE("BaseShapeGenerator/torus")
{
    auto tri = CADacomb::BaseShapeGenerator::torus(24, 12, 2.0f, 0.5f);
    REQUIRE(CADacomb::HalfEdgeHealth::isHealthy(tri));
    auto aabb = tri.getAABB();
    REQUIRE(aabb.first.z == Catch::Approx(-0.5));
    REQUIRE((aabb.first.z + aabb.second.z) == Catch::Approx(0.5));
}

TEST_CASE("BaseShapeGenerator/torusMinimal")
{
    auto tri = CADacomb::BaseShapeGenerator::torus(3, 3, 1.0f, 0.3f);
    REQUIRE(CADacomb::HalfEdgeHealth::isHealthy(tri));
}

TEST_CASE("BaseShapeGenerator/capsule")
{
    auto tri = CADacomb::BaseShapeGenerator::capsule(16, 6, 0.5f, 1.5f);
    REQUIRE(CADacomb::HalfEdgeHealth::isHealthy(tri));
    auto aabb = tri.getAABB();
    REQUIRE(aabb.first.z == Catch::Approx(-0.5));
    REQUIRE((aabb.first.z + aabb.second.z) == Catch::Approx(2.0));
}

TEST_CASE("BaseShapeGenerator/capsuleZeroHeight")
{
    auto tri = CADacomb::BaseShapeGenerator::capsule(12, 4, 1.0f, 0.0f);
    REQUIRE(CADacomb::HalfEdgeHealth::isHealthy(tri));
}

TEST_CASE("BaseShapeGenerator/prismHexagon")
{
    auto tri = CADacomb::BaseShapeGenerator::prism(6, 1.0f, 2.0f);
    REQUIRE(CADacomb::HalfEdgeHealth::isHealthy(tri));
}

TEST_CASE("BaseShapeGenerator/prismTriangle")
{
    auto tri = CADacomb::BaseShapeGenerator::prism(3, 1.0f, 1.0f);
    REQUIRE(CADacomb::HalfEdgeHealth::isHealthy(tri));
}

TEST_CASE("BaseShapeGenerator/octahedron")
{
    auto tri = CADacomb::BaseShapeGenerator::octahedron(2.0f);
    REQUIRE(CADacomb::HalfEdgeHealth::isHealthy(tri));
    REQUIRE(tri.vertices.size() == 6);
    REQUIRE(tri.indices.size() == 24);
}

TEST_CASE("BaseShapeGenerator/lBracket")
{
    auto tri = CADacomb::BaseShapeGenerator::lBracket(2.0f, 1.0f, 2.0f, 0.4f);
    REQUIRE(CADacomb::HalfEdgeHealth::isHealthy(tri));
}

TEST_CASE("BaseShapeGenerator/steppedBlock")
{
    auto tri = CADacomb::BaseShapeGenerator::steppedBlock(3.0f, 1.5f, 2.0f);
    REQUIRE(CADacomb::HalfEdgeHealth::isHealthy(tri));
}

TEST_CASE("BaseShapeGenerator/plateWithHoles")
{
    auto tri = CADacomb::BaseShapeGenerator::plateWithHoles(16, 4.0f, 2.0f, 0.3f, 0.3f);
    REQUIRE(CADacomb::HalfEdgeHealth::isHealthy(tri));
}

TEST_CASE("BaseShapeGenerator/plateWithHolesMinimal")
{
    auto tri = CADacomb::BaseShapeGenerator::plateWithHoles(4, 2.0f, 1.0f, 0.2f, 0.15f);
    REQUIRE(CADacomb::HalfEdgeHealth::isHealthy(tri));
}

TEST_CASE("BaseShapeGenerator/ConvertLBracketToCad")
{
    auto                                  tri      = CADacomb::BaseShapeGenerator::lBracket(2.0f, 1.0f, 2.0f, 0.4f);
    auto                                  clusters = CADacomb::Clustering::clusterByNormal(tri, 5.0f);
    CADacomb::CAD::KernelSm               kernel;
    CADacomb::CAD::ClusterToCadConverter  converter;
    std::unique_ptr<CADacomb::CAD::Shape> shape = converter.convert(kernel, tri, clusters, 1e-3, 1e-4, 1e-3, true);
    REQUIRE(shape != nullptr);
    auto aabb = shape->getAABB();
    REQUIRE(aabb.getMax().x == Catch::Approx(2.0).margin(1e-2));
    REQUIRE(aabb.getMax().y == Catch::Approx(1.0).margin(1e-2));
    REQUIRE(aabb.getMax().z == Catch::Approx(2.0).margin(1e-2));
}

TEST_CASE("BaseShapeGenerator/ConvertSteppedBlockToCad")
{
    auto                                  tri      = CADacomb::BaseShapeGenerator::steppedBlock(2.0f, 1.0f, 1.5f);
    auto                                  clusters = CADacomb::Clustering::clusterByNormal(tri, 5.0f);
    CADacomb::CAD::KernelSm               kernel;
    CADacomb::CAD::ClusterToCadConverter  converter;
    std::unique_ptr<CADacomb::CAD::Shape> shape = converter.convert(kernel, tri, clusters, 1e-3, 1e-4, 1e-3, true);
    REQUIRE(shape != nullptr);
    auto aabb = shape->getAABB();
    REQUIRE(aabb.getMax().x == Catch::Approx(2.0).margin(1e-2));
    REQUIRE(aabb.getMax().y == Catch::Approx(1.0).margin(1e-2));
    REQUIRE(aabb.getMax().z == Catch::Approx(1.5).margin(1e-2));
}

TEST_CASE("BaseShapeGenerator/ConvertOctahedronNaiveToCad")
{
    auto                                    tri = CADacomb::BaseShapeGenerator::octahedron(1.5f);
    CADacomb::CAD::KernelSm                 kernel;
    CADacomb::CAD::NaiveMeshToCadConverter  converter;
    std::unique_ptr<CADacomb::CAD::Shape>   shape = converter.convert(kernel, tri, 1e-4, true);
    REQUIRE(shape != nullptr);
    auto aabb = shape->getAABB();
    REQUIRE(aabb.getMin().x == Catch::Approx(-1.5).margin(1e-2));
    REQUIRE(aabb.getMax().x == Catch::Approx(1.5).margin(1e-2));
}
#endif

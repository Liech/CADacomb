#pragma once

#include "CADacomb/Triangle/Triangulation.h"
#include "HalfEdge.h"
#include <memory>

namespace Library
{
    class mesh2halfedge
    {
      public:
        static std::unique_ptr<HalfEdgeMesh>  convert(const Triangulation&);
        static std::unique_ptr<Triangulation> convert(const HalfEdgeMesh&);
    };
}
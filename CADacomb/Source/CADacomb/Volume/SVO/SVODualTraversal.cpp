#include "SVODualTraversal.h"

namespace CADacomb
{
    SVODualTraversal::SVODualTraversal() {}

    SVODualTraversal::~SVODualTraversal() {}

    int SVODualTraversal::getPosChild(SVOAxis ax, int k)
    {
        switch (ax)
        {
            case SVOAxis::X:
                return 1 | ((k & 1) ? 2 : 0) | ((k & 2) ? 4 : 0);
            case SVOAxis::Y:
                return 2 | ((k & 1) ? 1 : 0) | ((k & 2) ? 4 : 0);
            case SVOAxis::Z:
                return 4 | ((k & 1) ? 1 : 0) | ((k & 2) ? 2 : 0);
        }
        return 0;
    }

    int SVODualTraversal::getNegChild(SVOAxis ax, int k)
    {
        switch (ax)
        {
            case SVOAxis::X:
                return 0 | ((k & 1) ? 2 : 0) | ((k & 2) ? 4 : 0);
            case SVOAxis::Y:
                return 0 | ((k & 1) ? 1 : 0) | ((k & 2) ? 4 : 0);
            case SVOAxis::Z:
                return 0 | ((k & 1) ? 1 : 0) | ((k & 2) ? 2 : 0);
        }
        return 0;
    }
}

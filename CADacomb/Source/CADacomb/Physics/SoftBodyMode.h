#pragma once

namespace CADacomb::Physics
{
    // Selects the internal constraint topology used to simulate a deformable soft body.
    enum class SoftBodyMode
    {
        Balloon        = 0,
        InternalStruts = 1,
        Volumetric     = 2
    };
}

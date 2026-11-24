#pragma once

namespace CADacomb::Animation
{
    // Interpolation mode used when sampling between animation keyframes.
    enum class InterpolationMode
    {
        Step,
        Linear,
        CubicSpline
    };
}

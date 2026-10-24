#include "TessellationOptions.h"

namespace CADacomb::CAD
{
    TessellationOptions::TessellationOptions() {}

    TessellationOptions::~TessellationOptions() {}

    double TessellationOptions::getChordHeightTolerance() const
    {
        return chordHeightTolerance;
    }

    void TessellationOptions::setChordHeightTolerance(double tolerance)
    {
        chordHeightTolerance = tolerance;
    }

    double TessellationOptions::getSurfaceAngleToleranceDeg() const
    {
        return surfaceAngleToleranceDeg;
    }

    void TessellationOptions::setSurfaceAngleToleranceDeg(double toleranceDeg)
    {
        surfaceAngleToleranceDeg = toleranceDeg;
    }
}

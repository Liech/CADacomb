#pragma once

namespace CADacomb::CAD
{
    class TessellationOptions
    {
      public:
        TessellationOptions();
        virtual ~TessellationOptions();

        double getChordHeightTolerance() const;
        void   setChordHeightTolerance(double chordHeightTolerance);
        double getSurfaceAngleToleranceDeg() const;
        void   setSurfaceAngleToleranceDeg(double surfaceAngleToleranceDeg);

      private:
        double chordHeightTolerance     = 0.01;
        double surfaceAngleToleranceDeg = 15.0;
    };
}

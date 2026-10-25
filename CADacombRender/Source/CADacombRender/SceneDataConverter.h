#pragma once

#include "CADacomb/Volume/LODStrategy/SceneData.h"
#include "Vulcant/Toolkit/SceneData.h"

namespace CADacomb::Render
{
    class SceneDataConverter
    {
      public:
        SceneDataConverter();
        virtual ~SceneDataConverter();

        static CADacomb::SceneData         convert(const Vulcant::Toolkit::SceneData& sceneData);
        static Vulcant::Toolkit::SceneData convert(const CADacomb::SceneData& sceneData);
    };
}

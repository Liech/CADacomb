#pragma once

#include "CADacomb/Animation/Transform.h"

#include <glm/glm.hpp>
#include <string>

namespace CADacomb::Animation
{
    // Single named joint node in a Skeleton hierarchy, storing rest pose and inverse bind matrix.
    class Joint
    {
      public:
        Joint();
        Joint(const std::string& name, int parentIndex, const Transform& restLocalTransform);
        virtual ~Joint();

        const std::string& getName() const;
        void               setName(const std::string& name);

        int  getParentIndex() const;
        void setParentIndex(int parentIndex);

        const Transform& getRestLocalTransform() const;
        void             setRestLocalTransform(const Transform& restLocalTransform);

        const glm::dmat4& getInverseBindMatrix() const;
        void              setInverseBindMatrix(const glm::dmat4& inverseBindMatrix);

      private:
        std::string name;
        int         parentIndex = -1;
        Transform   restLocalTransform;
        glm::dmat4  inverseBindMatrix = glm::dmat4(1.0);
    };
}

#pragma once

#include "AnySVO.h"
#include <glm/glm.hpp>
#include <memory>

namespace Library::Volume
{
    class SVOResult
    {
      public:
        SVOResult();
        virtual ~SVOResult();

        SVOResult(SVOResult&& other) noexcept;
        SVOResult& operator=(SVOResult&& other) noexcept;

        SVOResult(const SVOResult&)            = delete;
        SVOResult& operator=(const SVOResult&) = delete;

        std::unique_ptr<AnySVO> tree           = nullptr;
        glm::dmat4              transformation = glm::dmat4(1.0);
    };
}

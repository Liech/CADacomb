cmake_minimum_required(VERSION 3.20...3.30)

project(CADacombRender)

find_package(glm REQUIRED)
if(NOT TARGET Vulcant::Vulcant)
    find_package(Vulcant CONFIG REQUIRED)
endif()

set(Source_Files
    "Source/CADacombRender/RenderHelper.cpp"
    "Source/CADacombRender/RenderHelper.h"
)
source_group("Source" FILES ${Source_Files})

set(ALL_FILES
    ${Source_Files}
)

add_library(CADacombRender ${ALL_FILES})
add_library(CADacomb::CADacombRender ALIAS CADacombRender)
set_property(TARGET CADacombRender PROPERTY CXX_STANDARD 20)

target_link_libraries(CADacombRender PUBLIC 
    CADacomb
    Vulcant::Vulcant
    glm::glm
)

target_include_directories(CADacombRender PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Source>
    $<INSTALL_INTERFACE:include>
)

install(TARGETS CADacombRender
    EXPORT CADacombTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

install(DIRECTORY Source/ DESTINATION include
    FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

if(CADACOMB_BUILD_TESTS)
    find_package(Catch2 REQUIRED)

    set(Test_Source_Files
        "Source/CADacombRender/testMain.cpp"
        ${Source_Files}
    )
    add_executable(CADacombRenderTest ${Test_Source_Files})
    set_property(TARGET CADacombRenderTest PROPERTY CXX_STANDARD 20)
    target_link_libraries(CADacombRenderTest PUBLIC 
        CADacomb
        Vulcant::Vulcant
        Catch2::Catch2
        glm::glm
    )
    target_include_directories(CADacombRenderTest PUBLIC 
        ${CMAKE_CURRENT_SOURCE_DIR}/Source
    )
    set_property(TARGET CADacombRenderTest PROPERTY COMPILE_DEFINITIONS ISTESTPROJECT=1)

    include(CTest)
    include(Catch)
    catch_discover_tests(CADacombRenderTest)
endif()

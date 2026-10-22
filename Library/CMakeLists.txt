cmake_minimum_required(VERSION 3.20...3.30)

project(CADacomb)

find_package(glm REQUIRED)
find_package(nlohmann_json REQUIRED)

set(Source_Files
    "Source/Library/HelloWorld.cpp"
    "Source/Library/HelloWorld.h"
)
source_group("Source" FILES ${Source_Files})

set(ALL_FILES
    ${Source_Files}
)

add_library(CADacomb ${ALL_FILES})
add_library(CADacomb::CADacomb ALIAS CADacomb)
set_property(TARGET CADacomb PROPERTY CXX_STANDARD 20)

target_link_libraries(CADacomb PUBLIC 
    glm::glm
    nlohmann_json::nlohmann_json
)

target_include_directories(CADacomb PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Source>
    $<INSTALL_INTERFACE:include>
)

install(TARGETS CADacomb
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
        "Source/Library/testMain.cpp"
        ${Source_Files}
    )
    add_executable(CADacombTest ${Test_Source_Files})
    set_property(TARGET CADacombTest PROPERTY CXX_STANDARD 20)
    target_link_libraries(CADacombTest PUBLIC 
        Catch2::Catch2
        glm::glm
        nlohmann_json::nlohmann_json
    )
    target_include_directories(CADacombTest PUBLIC 
        ${CMAKE_CURRENT_SOURCE_DIR}/Source
    )
    set_property(TARGET CADacombTest PROPERTY COMPILE_DEFINITIONS ISTESTPROJECT=1)

    include(CTest)
    include(Catch)
    catch_discover_tests(CADacombTest)
endif()

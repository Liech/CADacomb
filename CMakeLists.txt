cmake_minimum_required(VERSION 3.20...3.30)
project(CADacomb)

option(CADACOMB_BUILD_RENDER "Build CADacombRender rendering helper module" OFF)
option(CADACOMB_BUILD_TESTS "Build test executables" ON)

include(CMakePackageConfigHelpers)
include(GNUInstallDirs)

add_subdirectory(Library)

if(CADACOMB_BUILD_RENDER)
    add_subdirectory(CADacombRender)
endif()

install(EXPORT CADacombTargets
    FILE CADacombTargets.cmake
    NAMESPACE CADacomb::
    DESTINATION share/CADacomb
)

configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/CADacombConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/CADacombConfig.cmake"
    INSTALL_DESTINATION share/CADacomb
)

write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/CADacombConfigVersion.cmake"
    VERSION 0.0.1
    COMPATIBILITY SameMajorVersion
)

install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/CADacombConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/CADacombConfigVersion.cmake"
    DESTINATION share/CADacomb
)

if(CADACOMB_BUILD_TESTS)
    set_property(DIRECTORY PROPERTY VS_STARTUP_PROJECT CADacombTest)
endif()
set_property(GLOBAL PROPERTY USE_FOLDERS ON)
set(PREDEFINED_TARGETS_FOLDER "CustomTargets")

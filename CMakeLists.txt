cmake_minimum_required(VERSION 3.20...3.30)
project(CADacomb)

add_subdirectory(Library)

set_property(DIRECTORY PROPERTY VS_STARTUP_PROJECT CADacombTest)
set_property(GLOBAL PROPERTY USE_FOLDERS ON)
set(PREDEFINED_TARGETS_FOLDER "CustomTargets")

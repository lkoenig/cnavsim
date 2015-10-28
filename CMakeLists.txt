project(NavSim)
cmake_minimum_required(VERSION 3.0)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)


# Required packages
find_package(Eigen3 REQUIRED)
find_package(SDL REQUIRED)

# 
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${SDL_INCLUDE_DIR})

#
add_subdirectory(src)


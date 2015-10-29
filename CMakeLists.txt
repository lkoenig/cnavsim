project(NavSim)
cmake_minimum_required(VERSION 3.0)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)


# Required packages
find_package(Eigen3 REQUIRED)
find_package(SDL REQUIRED)

# Platform specific
if(APPLE)
    find_library(ICONV_LIBRARY iconv)
    find_library(CORE_AUDIO_FRAMEWORK CoreAudio)
    find_library(AUDIO_UNIT_FRAMEWORK AudioUnit)
    find_library(IO_KIT_FRAMEWORK IOKit)
    find_library(FORCE_FEEDBACK_FRAMEWORK ForceFeedback)
    find_library(CARBON_FRAMEWORK Carbon)
    
    list(APPEND EXTRA_LIBS 
        ${ICONV_LIBRARY} 
        ${IO_KIT_FRAMEWORK} 
        ${CORE_AUDIO_FRAMEWORK}
        ${AUDIO_UNIT_FRAMEWORK}
        ${FORCE_FEEDBACK_FRAMEWORK}
        ${CARBON_FRAMEWORK}
    )
endif()

# 
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${SDL_INCLUDE_DIR})

#
add_subdirectory(src)


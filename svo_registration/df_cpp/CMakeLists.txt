# FYP Phase 8: distance-field registration library (M3) + pybind11 module fyp_df.
# Standalone build, NOT part of the catkin workspace and NOT using SvoSetup.cmake: SVO's flags include
# -ffast-math / -fno-signed-zeros, which break Eigen 3.3.7's SSE Matrix4d::inverse
# (docs/pipeline_notes.md). This library uses plain -O3 (Release), no fast-math; the source refuses
# to compile if __FAST_MATH__ is defined.
# Build: svo_registration/df_cpp/build.sh (puts fyp_df.*.so next to the registration node scripts).
cmake_minimum_required(VERSION 3.16)
project(fyp_df CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
# System Python 3.8 (runs the ROS nodes); pybind11 headers/CMake config from the analysis venv.
set(PYBIND11_FINDPYTHON ON)
find_package(Python 3.8 EXACT REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11 CONFIG REQUIRED)

add_library(fyp_df_core STATIC src/df_registration.cpp)
target_include_directories(fyp_df_core PUBLIC include)
target_link_libraries(fyp_df_core PUBLIC Eigen3::Eigen)
target_compile_options(fyp_df_core PRIVATE -Wall -Wextra)

pybind11_add_module(fyp_df src/bindings.cpp)
target_link_libraries(fyp_df PRIVATE fyp_df_core)

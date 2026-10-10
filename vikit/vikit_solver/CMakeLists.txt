cmake_minimum_required(VERSION 3.16)
project(vikit_solver)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include(SvoSetup)

#############
# LIBRARIES #
#############
ament_auto_add_library(${PROJECT_NAME} SHARED
  include/vikit/solver/mini_least_squares_solver.h
  include/vikit/solver/implementation/mini_least_squares_solver.hpp
  src/robust_cost.cpp
  include/vikit/solver/robust_cost.h
)
target_link_libraries(${PROJECT_NAME} ${SVO_SYSTEM_LIBRARIES})

##########
# EXPORT #
##########
ament_auto_package()

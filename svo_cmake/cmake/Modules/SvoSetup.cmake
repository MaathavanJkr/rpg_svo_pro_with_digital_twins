# Common build settings for all SVO packages. Use with `include(SvoSetup)`.

IF(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  SET(CMAKE_BUILD_TYPE Release) # Release, RelWithDebInfo
ENDIF()
SET(CMAKE_VERBOSE_MAKEFILE OFF)

# rclcpp requires C++17.
SET(CMAKE_CXX_STANDARD 17)
SET(CMAKE_CXX_STANDARD_REQUIRED ON)
SET(CMAKE_CXX_EXTENSIONS OFF)
SET(CMAKE_POSITION_INDEPENDENT_CODE ON)

# user build settings
SET(USE_LOOP_CLOSING TRUE)
SET(USE_GLOBAL_MAP FALSE)

# Set definitions
IF(USE_LOOP_CLOSING)
  ADD_DEFINITIONS(-DSVO_LOOP_CLOSING)
ENDIF()

IF(USE_GLOBAL_MAP)
  ADD_DEFINITIONS(-DSVO_GLOBAL_MAP)
ENDIF()

ADD_DEFINITIONS(-DSVO_USE_ROS)
ADD_DEFINITIONS(-DSVO_USE_OPENGV)
ADD_DEFINITIONS(-DSVO_DEPTHFILTER_IN_REPROJECTOR)

#############################################################################
# Set build flags. ARM (e.g. Odroid, Jetson) is detected from the target processor,
# or forced by setting the ARM_ARCHITECTURE environment variable.
option(SVO_MARCH_NATIVE "Compile SVO with -march=native" ON)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -Wall -D_LINUX -D_REENTRANT -Wno-unused-variable -Wno-unused-but-set-variable -Wno-unknown-pragmas -Wno-unused-but-set-parameter -Wno-int-in-bool-context -Wno-maybe-uninitialized -Wno-unused-function -Wno-deprecated-declarations -Wno-class-memaccess -Wno-sign-compare -Wno-reorder")

IF(DEFINED ENV{ARM_ARCHITECTURE} OR CMAKE_SYSTEM_PROCESSOR MATCHES "^(arm|aarch64)")
  IF(CMAKE_SYSTEM_PROCESSOR MATCHES "^arm")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mfpu=neon -march=armv7-a")
  ENDIF()
  ADD_DEFINITIONS(-DHAVE_FAST_NEON)
ELSE()
  IF(SVO_MARCH_NATIVE)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")
  ENDIF()
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mmmx -msse -msse2 -msse3 -mssse3 -mno-avx")
ENDIF()
SET(CMAKE_CXX_FLAGS_RELEASE "-O3 -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops -ffast-math -fno-finite-math-only")

#############################################################################
# System dependencies shared by (almost) all SVO packages.
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(glog REQUIRED)
find_package(gflags REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS})

# Debian/Ubuntu install FindGeographicLib.cmake here (libgeographiclib-dev).
list(APPEND CMAKE_MODULE_PATH "/usr/share/cmake/geographiclib")

SET(SVO_SYSTEM_LIBRARIES
  glog::glog
  gflags
  ${OpenCV_LIBRARIES}
  ${YAML_CPP_LIBRARIES})

# Links the helpers used by the SVO unit tests (eigen-checks) to a test target.
function(svo_link_test_deps target)
  find_package(eigen_checks REQUIRED)
  target_link_libraries(${target} eigen_checks::eigen_checks)
endfunction()

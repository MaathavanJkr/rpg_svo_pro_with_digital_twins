cmake_minimum_required(VERSION 3.16)
project(svo_vio_common)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include(SvoSetup)

#############
# LIBRARIES #
#############
set(HEADERS
  include/svo/vio_common/logging.hpp
  include/svo/vio_common/matrix_operations.hpp
  include/svo/vio_common/matrix.hpp
  include/svo/vio_common/test_utils.hpp
  include/svo/vio_common/backend_types.hpp
  )

set(SOURCES
  src/vio_common.cpp
  src/test_utils.cpp
  )

ament_auto_add_library(${PROJECT_NAME} SHARED ${SOURCES} ${HEADERS})
target_link_libraries(${PROJECT_NAME} ${SVO_SYSTEM_LIBRARIES})

##########
# EXPORT #
##########
ament_auto_package()

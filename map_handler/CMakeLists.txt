# Copyright (c) 2024 PAL Robotics S.L. All rights reserved.
#
# Unauthorized copying of this file, via any medium is strictly prohibited,
# unless it was supplied under the terms of a license agreement or
# nondisclosure agreement with PAL Robotics SL. In this case it may not be
# copied or disclosed except in accordance with the terms of that agreement.

cmake_minimum_required(VERSION 3.8)
project(map_handler)

# Global flags
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

list(APPEND CMAKE_MODULE_PATH "${FIND_OPENVDB_PATH}")

find_package(OpenVDB REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Targets
ament_auto_add_library(${PROJECT_NAME}_lib
  src/map_handler/frustum_fov.cpp
)

target_link_libraries(${PROJECT_NAME}_lib
  ${Tbb_LIBRARIES}
  ${OpenVDB_LIBRARIES}
)

ament_auto_add_executable(frustum_debug
  src/frustum_debug.cpp
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(
  include
)

# Testing
if(BUILD_TESTING)
  ament_auto_find_test_dependencies()
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()

# Copyright (c) 2024 PAL Robotics S.L. All rights reserved.
#
# Unauthorized copying of this file, via any medium is strictly prohibited,
# unless it was supplied under the terms of a license agreement or
# nondisclosure agreement with PAL Robotics SL. In this case it may not be
# copied or disclosed except in accordance with the terms of that agreement.

cmake_minimum_required(VERSION 3.8)
project(representation_plugins)

# Global flags
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Targets
ament_auto_add_library(representation_plugins src/representation_plugins/plugin_test.cpp)
target_include_directories(representation_plugins PUBLIC
<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
<INSTALL_INTERFACE:include>)

ament_target_dependencies(
  representation_plugins
  representation_plugin_base
  pluginlib
)

install(
  TARGETS representation_plugins
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

pluginlib_export_plugin_description_file(representation_plugin_base plugins.xml)

# Testing
if(BUILD_TESTING)
  ament_auto_find_test_dependencies()
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_libraries(
  representation_plugins
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_auto_package()

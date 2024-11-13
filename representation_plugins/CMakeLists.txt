cmake_minimum_required(VERSION 3.8)
project(representation_plugins)

add_compile_options(-fPIC)

# Global flags
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

list(APPEND CMAKE_MODULE_PATH "${FIND_OPENVDB_PATH}")

find_package(ament_cmake_auto REQUIRED)
find_package(map_handler REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenVDB REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(representation_plugin_base REQUIRED)
ament_auto_find_build_dependencies()

message("${INCLUDE_DIRS}")

# Targets
add_library(representation_plugins SHARED 
src/representation_plugins/semantic_plugin.cpp
src/representation_plugins/plugin_test.cpp
src/representation_plugins/plugin_box_test.cpp
src/representation_plugins/plugin_another_box_test.cpp)

target_link_libraries(${PROJECT_NAME}
  ${PCL_LIBRARIES}  
  ${OpenVDB_LIBRARIES}
)

pluginlib_export_plugin_description_file(representation_plugins plugins.xml)

target_include_directories(representation_plugins PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)

ament_target_dependencies(
  representation_plugins
  rclcpp
  representation_plugin_base
  pcl_conversions
  pcl_ros
  map_handler
  pluginlib
  vdb2pc
)

ament_export_include_directories(
  include
)

ament_export_libraries(
  representation_plugins
)

ament_export_targets(
  export_${PROJECT_NAME}
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

# Testing
if(BUILD_TESTING)
  ament_auto_find_test_dependencies()
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()

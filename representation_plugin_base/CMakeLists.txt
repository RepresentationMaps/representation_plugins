cmake_minimum_required(VERSION 3.8)
project(representation_plugin_base)

add_compile_options(-fPIC)

# Global flags
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(TBB REQUIRED)
ament_auto_find_build_dependencies()

# rosidl_generate_interfaces(${PROJECT_NAME}
#   "msg/StringArray.msg"
#   "msg/RegionsRegisterDescriptor.msg"
# )

# Targets
ament_auto_add_library(${PROJECT_NAME}_lib SHARED
  src/representation_plugin_base/plugin_base.cpp
  src/representation_plugin_base/regions_register.cpp
)

target_link_libraries(${PROJECT_NAME}_lib
  ${Tbb_LIBRARIES}
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}_lib
)
ament_export_dependencies(
  rclcpp
  pluginlib
)

# Testing
if(BUILD_TESTING)
  ament_auto_find_test_dependencies()
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()

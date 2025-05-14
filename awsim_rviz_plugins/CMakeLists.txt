cmake_minimum_required(VERSION 3.10)

project(awsim_rviz_plugins)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wnon-virtual-dtor -Woverloaded-virtual)
endif()

# Tests currently only run on OS X @ OSRF jenkins
# Enable on Linux by providing a display, enable on Windows via EnableDisplayTests=True
option(EnableDisplayTests "EnableDisplayTests")
set(DisplayTests "False" CACHE STRING "DisplayTestsVariable")

if(DEFINED ENV{DISPLAY})
  set(DISPLAYPRESENT TRUE)
endif()

if(MSVC)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj")
endif()

# We specifically don't turn on CMAKE_AUTOMOC, since it generates one huge
# mocs_compilation.cpp file that takes a lot of memory to compile.  Instead
# we create individual moc files that can be compiled separately.

find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_default_plugins REQUIRED)

find_package(geometry_msgs REQUIRED)

find_package(rclcpp REQUIRED)

set(awsim_rviz_plugins_headers_to_moc
  include/ego_placement/ego_placement.hpp
  include/npc_spawner/npc_spawner.hpp
  include/npc_spawner/npc_spawner_status.hpp
)

foreach(header "${awsim_rviz_plugins_headers_to_moc}")
  qt5_wrap_cpp(awsim_rviz_plugins_moc_files "${header}")
endforeach()

set(awsim_rviz_plugins_source_files
  src/ego_placement/ego_placement.cpp
  src/npc_spawner/npc_spawner.cpp
  src/npc_spawner/npc_spawner_status.cpp
)

add_library(awsim_rviz_plugins SHARED
  ${awsim_rviz_plugins_moc_files}
  ${awsim_rviz_plugins_source_files}
)

target_include_directories(awsim_rviz_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_link_libraries(awsim_rviz_plugins PUBLIC
  ${geometry_msgs_TARGETS}
  rclcpp::rclcpp
  rviz_common::rviz_common
  rviz_rendering::rviz_rendering
  rviz_default_plugins::rviz_default_plugins
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(awsim_rviz_plugins PRIVATE "awsim_rviz_plugins_BUILDING_LIBRARY")

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

# Export old-style CMake variables
ament_export_include_directories("include/${PROJECT_NAME}")

# Export modern CMake targets
ament_export_targets(awsim_rviz_plugins HAS_LIBRARY_TARGET)

ament_export_dependencies(
  geometry_msgs
  rclcpp
  rviz_common
  rviz_rendering
  rviz_default_plugins
)

install(
  TARGETS awsim_rviz_plugins
  EXPORT awsim_rviz_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_package(
  CONFIG_EXTRAS "awsim_rviz_plugins-extras.cmake"
)

if(TEST cpplint)
  set_tests_properties(cpplint PROPERTIES TIMEOUT 180)
endif()

if(TEST copyright)
  set_tests_properties(copyright PROPERTIES TIMEOUT 600)
endif()

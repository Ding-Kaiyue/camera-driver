cmake_minimum_required(VERSION 3.16)
project(realsense_voxblox_bridge LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(BUILD_BRIDGE "Build RealSense to Voxblox ESDF bridge" ON)
set(VOXBLOX_SOURCE_DIR "$ENV{HOME}/Projects/voxblox/voxblox" CACHE PATH "Path to voxblox core source dir")

if(NOT EXISTS "${VOXBLOX_SOURCE_DIR}/CMakeLists.txt")
  message(FATAL_ERROR "VOXBLOX_SOURCE_DIR is invalid: ${VOXBLOX_SOURCE_DIR}")
endif()

find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(REALSENSE2 REQUIRED realsense2)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_subdirectory("${VOXBLOX_SOURCE_DIR}" "${CMAKE_BINARY_DIR}/_deps/voxblox-build")

if(BUILD_BRIDGE)
  add_library(realsense_voxblox_bridge_core
    src/config/config_helpers.cpp
    src/config/app_config_yaml.cpp
    src/config/camera_extrinsics_yaml.cpp
    src/bridge/bridge_app.cpp
    src/bridge/realsense_pointcloud_io.cpp
    src/bridge/voxblox_mapper.cpp
  )

  target_include_directories(realsense_voxblox_bridge_core
    PUBLIC
      include
    PRIVATE
      ${REALSENSE2_INCLUDE_DIRS}
  )

  target_link_directories(realsense_voxblox_bridge_core
    PRIVATE
      ${REALSENSE2_LIBRARY_DIRS}
  )

  target_link_libraries(realsense_voxblox_bridge_core
    PUBLIC
      Eigen3::Eigen
      voxblox::core
      ${REALSENSE2_LIBRARIES}
  )

  target_link_libraries(realsense_voxblox_bridge_core
    PUBLIC
      rclcpp::rclcpp
      sensor_msgs::sensor_msgs_library
  )

  target_compile_options(realsense_voxblox_bridge_core PRIVATE
    -Wall
    -Wextra
    -Wno-range-loop-construct
    ${REALSENSE2_CFLAGS_OTHER}
  )

  add_executable(realsense_voxblox_bridge
    src/main.cpp
  )

  target_include_directories(realsense_voxblox_bridge
    PRIVATE
      include
  )

  target_link_libraries(realsense_voxblox_bridge
    PRIVATE
      realsense_voxblox_bridge_core
  )

  target_compile_options(realsense_voxblox_bridge PRIVATE
    -Wall
    -Wextra
  )

  add_executable(esdf_query_example
    example/example.cpp
  )

  target_link_libraries(esdf_query_example
    PRIVATE
      Eigen3::Eigen
      voxblox::core
  )

  target_compile_options(esdf_query_example PRIVATE
    -Wall
    -Wextra
  )
endif()

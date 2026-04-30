cmake_minimum_required(VERSION 3.16)
project(camera_driver LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(BUILD_BRIDGE "Build RealSense realtime pointcloud bridge" ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(REALSENSE2 REQUIRED IMPORTED_TARGET realsense2)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(controller_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

if(BUILD_BRIDGE)
  add_library(camera_driver_core SHARED
    src/config/camera_driver_config_yaml.cpp
    src/config/camera_config_yaml.cpp
    src/config/filter_config_yaml.cpp
    src/config/esdf_config_yaml.cpp
    src/application/camera_driver_application.cpp
    src/application/camera_driver_service.cpp
    src/application/camera_driver_interface.cpp
    src/camera/realsense_depth_source.cpp
    src/point_cloud/filter.cpp
    src/point_cloud/point_cloud_build.cpp
    src/point_cloud/point_cloud_publish.cpp
    src/esdf/live_esdf_registry.cpp
    src/esdf/esdf_publish.cpp
    src/esdf/esdf_map_ros.cpp
    src/esdf/remani_grid_map.cpp
  )

  target_include_directories(camera_driver_core
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
    PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/src
  )

  target_link_libraries(camera_driver_core
    rclcpp::rclcpp
    ament_index_cpp::ament_index_cpp
    sensor_msgs::sensor_msgs_library
    geometry_msgs::geometry_msgs__rosidl_typesupport_cpp
    tf2_ros::tf2_ros
    yaml-cpp
    Eigen3::Eigen
    PkgConfig::REALSENSE2
  )

  target_compile_options(camera_driver_core PRIVATE
    -Wall
    -Wextra
    -Wno-range-loop-construct
    ${REALSENSE2_CFLAGS_OTHER}
  )

  ament_target_dependencies(camera_driver_core
    ament_index_cpp
    rclcpp
    sensor_msgs
    geometry_msgs
    controller_interfaces
    tf2_ros
  )

  add_executable(camera_driver_example
    example/example.cpp
  )

  target_link_libraries(camera_driver_example
    PRIVATE
      camera_driver_core
  )

  target_compile_options(camera_driver_example PRIVATE
    -Wall
    -Wextra
  )

  add_executable(camera_driver_node
    src/node/camera_driver_node.cpp
  )

  target_link_libraries(camera_driver_node
    PRIVATE
      camera_driver_core
  )

  target_compile_options(camera_driver_node PRIVATE
    -Wall
    -Wextra
  )

  install(
    DIRECTORY include/camera_driver/
    DESTINATION include/camera_driver
  )

  install(
    DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
  )

  install(
    TARGETS camera_driver_core
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )

  install(
    TARGETS camera_driver_example
            camera_driver_node
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )

  install(
    EXPORT export_${PROJECT_NAME}
    NAMESPACE ${PROJECT_NAME}::
    DESTINATION share/${PROJECT_NAME}/cmake
  )

  ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
  ament_export_dependencies(
    ament_index_cpp
    Eigen3
    rclcpp
    sensor_msgs
    geometry_msgs
    controller_interfaces
    tf2_ros
  )
endif()

ament_package()

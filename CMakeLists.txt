cmake_minimum_required(VERSION 3.5) # 适用于ROS 2的CMake版本
project(finav) # 项目名称

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED) # 用于ROS 2的ament_cmake
find_package(Boost REQUIRED COMPONENTS thread serialization) # Boost库
find_package(rclcpp REQUIRED) # ROS 2的C++客户端库
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(std_msgs REQUIRED) # 标准消息类型
find_package(sensor_msgs REQUIRED) # 传感器消息类型
find_package(geometry_msgs REQUIRED) # 几何消息类型
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(slam_toolbox REQUIRED)
find_package(nav2_map_server REQUIRED)
find_package(TBB REQUIRED)

option(FINAV_BUILD_AVOIDANCE "Build independent avoidance observation node" ON)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FollowTarget.msg" "msg/AvoidanceRequest.msg" "msg/AvoidanceResult.msg" "msg/FusedScan.msg"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs)
ament_export_dependencies(rosidl_default_runtime)
if(FINAV_BUILD_AVOIDANCE)
  find_package(mrpt-nav 2.4.2 EXACT REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(diagnostic_msgs REQUIRED)
  find_package(interfaces REQUIRED)
  add_library(finav_avoidance_core src/avoidance/core.cpp)
  target_compile_features(finav_avoidance_core PUBLIC cxx_std_17)
  target_include_directories(finav_avoidance_core PUBLIC include)
  target_link_libraries(finav_avoidance_core PUBLIC mrpt::nav)
  add_executable(obstacle_avoidance_node src/rosnode/obstacle_avoidance_node.cpp)
  ament_target_dependencies(obstacle_avoidance_node
    rclcpp std_msgs sensor_msgs geometry_msgs nav_msgs diagnostic_msgs interfaces)
  rosidl_get_typesupport_target(avoidance_typesupport ${PROJECT_NAME} rosidl_typesupport_cpp)
  target_link_libraries(obstacle_avoidance_node finav_avoidance_core "${avoidance_typesupport}")
  install(TARGETS finav_avoidance_core DESTINATION lib)
  install(TARGETS obstacle_avoidance_node DESTINATION lib/${PROJECT_NAME})
  install(FILES scripts/control/avoidance_model.py
    DESTINATION lib/${PROJECT_NAME})
endif()

set(dependencies
  rclcpp
  rclcpp_lifecycle
  lifecycle_msgs
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
  message_filters
  laser_geometry
  )

# Hinson HE-3051 lidar driver (adapted from the vendor ROS 2 package).
set(hinson_he_driver_dir
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/hinson_he_lidar/hi_driver
)

add_executable(hins_he_lidar_node
  ${hinson_he_driver_dir}/src/main.cpp
  ${hinson_he_driver_dir}/src/XingSongDriver/xingsong_driver.cpp
  ${hinson_he_driver_dir}/src/XingSongDriver/laserprotocol.cpp
  ${hinson_he_driver_dir}/src/XingSongDriver/shadowsfilter.cpp
  ${hinson_he_driver_dir}/src/ConnectDriver/ConnectDriver.cpp
)

ament_target_dependencies(hins_he_lidar_node
  rclcpp
  sensor_msgs
  geometry_msgs
)

target_include_directories(hins_he_lidar_node PUBLIC
  $<BUILD_INTERFACE:${hinson_he_driver_dir}/src>
  $<BUILD_INTERFACE:${hinson_he_driver_dir}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(hins_he_lidar_node
  Boost::thread
)

add_executable(scan_fusion_node
  src/rosnode/scan_fusion_node.cpp
)

ament_target_dependencies(scan_fusion_node ${dependencies})
rosidl_get_typesupport_target(scan_typesupport ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(scan_fusion_node
  ${Boost_LIBRARIES}
  "${scan_typesupport}"
)

add_executable(fixed_localization_node src/rosnode/fixed_localization_node.cpp)
target_compile_features(fixed_localization_node PRIVATE cxx_std_17)
ament_target_dependencies(fixed_localization_node slam_toolbox nav2_map_server)
# Humble STB exports the RViz target separately from its legacy core library list.
foreach(stb_library localization_slam_toolbox toolbox_common kartoSlamToolbox slam_toolbox__rosidl_typesupport_cpp)
  find_library(FINAV_${stb_library}_LIBRARY NAMES ${stb_library}
    HINTS "${slam_toolbox_DIR}/../../../lib" REQUIRED)
  target_link_libraries(fixed_localization_node ${FINAV_${stb_library}_LIBRARY})
endforeach()
target_link_libraries(fixed_localization_node Boost::thread Boost::serialization TBB::tbb)

add_executable(continued_mapping_node src/rosnode/continued_mapping_node.cpp)
target_compile_features(continued_mapping_node PRIVATE cxx_std_17)
ament_target_dependencies(continued_mapping_node slam_toolbox std_msgs nav2_map_server)
find_library(FINAV_CONTINUED_MAPPING_LIBRARY NAMES map_and_localization_slam_toolbox
  HINTS "${slam_toolbox_DIR}/../../../lib" REQUIRED)
target_link_libraries(continued_mapping_node ${FINAV_CONTINUED_MAPPING_LIBRARY}
  ${FINAV_localization_slam_toolbox_LIBRARY} ${FINAV_toolbox_common_LIBRARY}
  ${FINAV_kartoSlamToolbox_LIBRARY} ${FINAV_slam_toolbox__rosidl_typesupport_cpp_LIBRARY}
  Boost::thread Boost::serialization TBB::tbb)

# Install launch files.
set(finav_share_dirs
  launch
  config
  rviz
  urdf
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/maps")
  list(APPEND finav_share_dirs maps)
endif()

install(DIRECTORY
  ${finav_share_dirs}
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python scripts
install(PROGRAMS
  scripts/control/person_follow.py
  scripts/service/run_vision.sh
  scripts/control/nav_voice_bridge.py
  scripts/control/nav_path_plan.py
  scripts/control/nav_task_manager.py
  scripts/control/nav_control.py
  scripts/control/nav_relocate.py
  scripts/imu/dm_imu_publisher.py
  scripts/tool/TF.sh
  scripts/tool/clean_nav.sh
  scripts/tool/clean_map.sh
  scripts/tool/save_map.sh
  scripts/tool/save_map.py
  scripts/map_location/location_visualizer.py
  start_finav.sh
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  scripts/control/ultrasonic_gate.py
  scripts/map_location/location_utils.py
  scripts/map_location/editor_server.py
  scripts/map_location/editor_map_io.py
  scripts/map_location/editor_store.py
  scripts/map_location/editor_geometry.py
  scripts/map_location/editor_routes.py
  scripts/map_location/editor_export.py
  scripts/control/planning_constraints.py
  scripts/control/route_tasks.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  scripts/control/path_planning/
  DESTINATION lib/${PROJECT_NAME}/path_planning
  FILES_MATCHING PATTERN "*.py"
)

# Install IMU Python modules (dm_serial, dm_crc, etc.)
install(DIRECTORY scripts/vision/ DESTINATION lib/${PROJECT_NAME}/vision
  FILES_MATCHING PATTERN "*.py")

install(DIRECTORY
  third_party/dm_imu/dm_imu_modules/
  DESTINATION lib/${PROJECT_NAME}/dm_imu_modules
  FILES_MATCHING PATTERN "*.py"
)

install(TARGETS hins_he_lidar_node scan_fusion_node fixed_localization_node continued_mapping_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(${dependencies})

ament_package()

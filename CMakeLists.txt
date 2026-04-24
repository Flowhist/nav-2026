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
find_package(Boost REQUIRED COMPONENTS thread) # Boost库
find_package(rclcpp REQUIRED) # ROS 2的C++客户端库
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(std_msgs REQUIRED) # 标准消息类型
find_package(sensor_msgs REQUIRED) # 传感器消息类型
find_package(geometry_msgs REQUIRED) # 几何消息类型

set(dependencies
  rclcpp
  rclcpp_lifecycle
  lifecycle_msgs
  std_msgs
  sensor_msgs
  geometry_msgs
  )

# Declare a C++ executable for free_lidar_node
add_executable(free_lidar_node
  FREE_CH_02.3.01.001_ROS2/FREE_CH_02.3.01.001/src/rosnode/free_lidar_node.cpp
  FREE_CH_02.3.01.001_ROS2/FREE_CH_02.3.01.001/src/driver/free_eth_driver.cpp
  FREE_CH_02.3.01.001_ROS2/FREE_CH_02.3.01.001/src/driver/trailing_filter.cpp
)

ament_target_dependencies(free_lidar_node ${dependencies})

target_include_directories(free_lidar_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/FREE_CH_02.3.01.001_ROS2/FREE_CH_02.3.01.001/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(free_lidar_node
  ${Boost_LIBRARIES}
)

# Declare a C++ executable for serial_bridge_node
# add_executable(serial_bridge_node 
#   src/rosnode/serial_bridge_node.cpp
# )

# ament_target_dependencies(serial_bridge_node ${dependencies})

# target_include_directories(serial_bridge_node PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)

# target_link_libraries(serial_bridge_node
#   ${Boost_LIBRARIES}
# )

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

install(
  DIRECTORY sim
  DESTINATION share/${PROJECT_NAME}/
  PATTERN "__pycache__" EXCLUDE
)

# Install Python scripts
install(PROGRAMS
  scripts/control/path_plan.py
  scripts/control/nav_control.py
  scripts/control/base_control.py
  scripts/control/js_kb_router.py
  scripts/control/joystick_control.py
  scripts/imu/dm_imu_publisher.py
  scripts/tool/TF.sh
  scripts/tool/save_map.sh
  scripts/tool/clean_nav.sh
  scripts/tool/clean_map.sh
  scripts/tool/manual_save_map.py
  sim/scripts/sim_chassis_sensor_pub.py
  sim/scripts/sim_gazebo_drive.py
  sim/scripts/sim_plan_visualizer.py
  sim/scripts/sim_send_goal.py
  sim/scripts/gz_topic_adapter.py
  start_finav.sh
  DESTINATION lib/${PROJECT_NAME}
)

# Install IMU Python modules (dm_serial, dm_crc, etc.)
install(DIRECTORY
  scripts/imu/dm_imu_modules/
  DESTINATION lib/${PROJECT_NAME}/dm_imu_modules
  FILES_MATCHING PATTERN "*.py"
)

install(TARGETS free_lidar_node # serial_bridge_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(${dependencies})

ament_package()

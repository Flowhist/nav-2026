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
find_package(std_msgs REQUIRED) # 标准消息类型
find_package(sensor_msgs REQUIRED) # 传感器消息类型
find_package(geometry_msgs REQUIRED) # 几何消息类型

set(dependencies
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  )
  
  # Declare a C++ executable for r2000_node
add_executable(r2000_node 
  r2000/src/r2000_node.cpp
  r2000/src/r2000_driver.cpp
  r2000/src/http_command_interface.cpp
  r2000/src/scan_data_receiver.cpp
)

ament_target_dependencies(r2000_node ${dependencies})

target_include_directories(r2000_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/r2000/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(r2000_node
  #r2000_driver
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
install(DIRECTORY
  launch
  config
  rviz
  maps
  urdf
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY sim
  DESTINATION share/${PROJECT_NAME}/
  PATTERN "launch" EXCLUDE
  PATTERN "__pycache__" EXCLUDE
)

# Install Python scripts
install(PROGRAMS
  scripts/general/base_control.py
  scripts/general/js_kb_router.py
  scripts/general/scan_angle_filter.py
  scripts/control/nav/nav_control.py
  scripts/control/nav/path_plan.py
  sim/scripts/sim_fake_pose_tf.py
  sim/scripts/sim_send_goal.py
  scripts/imu/dm_imu_publisher.py
  scripts/map/manual_save_map.py
  scripts/control/joystick/joystick_control.py
  start_finav.sh
  scripts/tool/save_map.sh
  scripts/tool/TF.sh
  scripts/tool/clean_map.sh
  scripts/tool/clean_nav.sh
  DESTINATION lib/${PROJECT_NAME}
)

# Install IMU Python modules (dm_serial, dm_crc, etc.)
install(DIRECTORY
  scripts/imu/dm_imu_modules/
  DESTINATION lib/${PROJECT_NAME}/dm_imu_modules
  FILES_MATCHING PATTERN "*.py"
)

install(TARGETS r2000_node # serial_bridge_node
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

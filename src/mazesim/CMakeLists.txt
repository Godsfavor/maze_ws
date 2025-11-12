cmake_minimum_required(VERSION 3.8)
project(mazesim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)



# Find additional dependencies
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Add nav2_command_translator executable
add_executable(nav2_command_translator src/nav2_command_translator.cpp)
ament_target_dependencies(nav2_command_translator
  rclcpp
  rclcpp_action
  nav2_msgs
  example_interfaces
  geometry_msgs
  tf2
  tf2_geometry_msgs
)
target_include_directories(nav2_command_translator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(nav2_command_translator PUBLIC c_std_99 cxx_std_17)

# Install the executable
install(TARGETS nav2_command_translator
  DESTINATION lib/${PROJECT_NAME})

  

# Robot controller executable (original direct control)
add_executable(robot_controller src/robot_controller.cpp)
ament_target_dependencies(robot_controller 
  rclcpp 
  geometry_msgs 
  nav_msgs 
  sensor_msgs
  example_interfaces
  tf2_ros
)
target_include_directories(robot_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS robot_controller
  DESTINATION lib/${PROJECT_NAME})


# Install Python scripts
install(PROGRAMS
  mazesim/odom_tf_broadcaster.py
  mazesim/odom_frame_fixer.py
  mazesim/laser_frame_remapper.py
  mazesim/map_loader.py
  DESTINATION lib/${PROJECT_NAME}
)



# Nav2 Command Bridge executable (NEW - Nav2 integration)
add_executable(nav2_command_bridge src/nav2_command_bridge.cpp)
ament_target_dependencies(nav2_command_bridge 
  rclcpp 
  rclcpp_action
  geometry_msgs 
  example_interfaces
  nav2_msgs
  tf2_ros
)
target_include_directories(nav2_command_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS nav2_command_bridge
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install map files
install(DIRECTORY maps
  DESTINATION share/${PROJECT_NAME}/
)

# Install world files
install(DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}/
)

# Install model files
install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME}/
)

# Install RViz config files
install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

# Install urdf files
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}/
)

# Install scripts
install(PROGRAMS
  scripts/generate_walls.py
  DESTINATION lib/${PROJECT_NAME}
)

# Add verification tool
add_executable(verify_positions src/verify_positions.cpp)
ament_target_dependencies(verify_positions rclcpp)
target_include_directories(verify_positions PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS verify_positions
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
cmake_minimum_required(VERSION 3.8)
project(robot_chase)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_compile_options(-std=c++17 -O3 -ffast-math)

include_directories(/opt/ros/humble/include)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Include Cpp "include" directory
include_directories(/opt/ros/humble/include)

include_directories(/opt/ros/humble/include/geometry_msgs)
include_directories(/opt/ros/humble/include/std_msgs)
include_directories(/opt/ros/humble/include/nav_msgs)
include_directories(/opt/ros/humble/include/tf2)
include_directories(/opt/ros/humble/include/tf2_geometry_msgs)
include_directories(/opt/ros/humble/include/tf2_ros)
include_directories(/opt/ros/humble/include/tf2_msgs)
include_directories(/opt/ros/humble/include/visualization_msgs)
include_directories(/opt/ros/humble/include/sensor_msgs)

add_executable(cpp_executable src/simulator_node.cpp)
target_include_directories(cpp_executable PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(cpp_executable PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  cpp_executable
  "rclcpp"
  "Eigen3"
  "geometry_msgs"
  "tf2"
  "tf2_ros"
  "nav_msgs"
  "std_msgs"
  "sensor_msgs"
  "visualization_msgs"
)


# # Create Cpp executable
# add_executable(cpp_executable src/freyja_simulator_node.cpp)
# ament_target_dependencies(cpp_executable rclcpp)

# Install Cpp executables
install(TARGETS
  cpp_executable
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/blue_vel_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/red_vel_node.py
  DESTINATION lib/${PROJECT_NAME}
)

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

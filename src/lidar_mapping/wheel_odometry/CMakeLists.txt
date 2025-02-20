cmake_minimum_required(VERSION 3.8)
project(wheel_odometry)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

include_directories(
  include
)

set(DEPENDENCIES
  "rclcpp"
  "sensor_msgs"
  "nav_msgs"
  "tf2"
  "tf2_ros"
  "geometry_msgs"
)

set(EXECUTABLE_NAME "wheel_odometry")

add_executable(${EXECUTABLE_NAME} src/main.cpp src/wheel_odometry.cpp)
ament_target_dependencies(${EXECUTABLE_NAME} ${DEPENDENCIES})

install(DIRECTORY param launch
  DESTINATION share/${PROJETCT_NAME}
)

install(TARGETS ${EXECUTABLE_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

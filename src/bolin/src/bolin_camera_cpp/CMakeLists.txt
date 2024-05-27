cmake_minimum_required(VERSION 3.8)
project(bolin_camera_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# include directories
include_directories(include ${OpenCV_INCLUDE_DIRS})

# create executable
add_executable(bolin_camera_cpp src/bolin_camera_cpp.cpp)

# link libraries
ament_target_dependencies(bolin_camera_cpp rclcpp sensor_msgs std_msgs OpenCV)

# install executable
install(TARGETS bolin_camera_cpp
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_lint_auto_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

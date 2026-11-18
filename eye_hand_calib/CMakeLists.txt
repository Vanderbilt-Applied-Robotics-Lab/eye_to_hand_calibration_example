cmake_minimum_required(VERSION 3.8)
project(eye_hand_calib)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(ament_index_cpp REQUIRED)

## INCLUDE(S)
include_directories(include)

# executables
add_executable(eye_hand_calib src/eye_hand_calib.cpp)

# dependencies
ament_target_dependencies(eye_hand_calib
  rclcpp
  OpenCV
  ament_index_cpp
)

install(
  DIRECTORY images
  DESTINATION share/${PROJECT_NAME}
)
install(TARGETS eye_hand_calib
  DESTINATION lib/${PROJECT_NAME})

ament_package()

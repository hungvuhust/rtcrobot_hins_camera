cmake_minimum_required(VERSION 3.8)
project(rtcrobot_hins_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 20)

set(dependencies
  ament_cmake
  rclcpp
  std_msgs
  tf2
  tf2_ros
  geometry_msgs
  rtcrobot_navutil
)

# find dependencies
foreach(dep ${dependencies})
  find_package(${dep} REQUIRED)
endforeach(dep)

include_directories(include)

add_executable(hins_camera_node
  src/main.cpp
  src/rtcrobot_hins_camera.cpp
)

ament_target_dependencies(hins_camera_node ${dependencies})

install(TARGETS
  hins_camera_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

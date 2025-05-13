cmake_minimum_required(VERSION 3.5)
project(seek_thermal)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(OpenCV REQUIRED)

find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(camera_calibration_parsers REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_proc REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(seek_thermal_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

###########
## Build ##
###########
# c++17 support required (11 not sufficient for pcl)
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++17" COMPILER_SUPPORTS_CXX14)
if(COMPILER_SUPPORTS_CXX14)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
else()
  message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++17 support. Please use a different C++ compiler.")
endif()

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${OpenCV_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
)

## Declare a C++ executable
## The recommended prefix ensures that target names across packages don't collide
add_executable(seek_thermal_node src/seek_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(seek_thermal_node
  seekcamera
  ${OpenCV_LIBRARIES}
)

ament_target_dependencies(seek_thermal_node
  ament_index_cpp
  rclcpp
  camera_calibration_parsers
  cv_bridge
  sensor_msgs
  std_msgs
  seek_thermal_msgs
  tf2_ros
)

#############
## Install ##
#############


## Mark libraries for installation
install(TARGETS 
  seek_thermal_node
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rclcpp)
ament_export_dependencies(camera_calibration_parsers)
ament_export_dependencies(cv_bridge)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(ament_cmake)
ament_export_dependencies(std_msgs)

ament_export_libraries(seek_thermal_node)

ament_package()

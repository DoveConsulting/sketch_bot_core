cmake_minimum_required(VERSION 3.8)
project(sketch_bot_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(xacro REQUIRED)
set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  rclcpp
  std_msgs
  geometry_msgs
  trajectory_msgs
  sensor_msgs
  visualization_msgs
  geometric_shapes
  moveit
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  Eigen3
  tf2
  tf2_ros
  tf2_eigen
  Eigen3
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
  list(APPEND ament_INCLUDE_DIRS ${${Dependency}_INCLUDE_DIRS})
endforeach()

include_directories(
  include ${ament_INCLUDE_DIRS}
)

# Define library source files
set(${PROJECT_NAME}_SRCS
  src/robot_control.cpp
  src/gcode.cpp
)

# Define header file
set(${PROJECT_NAME}_HDRS
  include/${PROJECT_NAME}/robot_control.h
  include/${PROJECT_NAME}/gcode.h
)

## Add library
add_library(${PROJECT_NAME} SHARED
  ${${PROJECT_NAME}_SRCS}
  ${${PROJECT_NAME}_HDRS}
)

ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_include_directories(${PROJECT_NAME} PUBLIC
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY src launch rviz 
  DESTINATION share/${PROJECT_NAME}
)

# Export targets and dependencies
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(${PROJECT_NAME})

# Add ROS nodes
add_executable(sketch src/sketch.cpp)
ament_target_dependencies(sketch ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(sketch ${PROJECT_NAME})

install(TARGETS
  sketch
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


ament_package()
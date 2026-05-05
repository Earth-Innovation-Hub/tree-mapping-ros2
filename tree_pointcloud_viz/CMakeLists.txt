cmake_minimum_required(VERSION 3.14)
project(tree_pointcloud_viz)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)

add_executable(tree_pointcloud_viz_node src/tree_pointcloud_viz_node.cpp)
ament_target_dependencies(tree_pointcloud_viz_node
  rclcpp
  sensor_msgs
  pcl_conversions
)
target_include_directories(tree_pointcloud_viz_node PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(tree_pointcloud_viz_node ${PCL_LIBRARIES})

install(TARGETS tree_pointcloud_viz_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

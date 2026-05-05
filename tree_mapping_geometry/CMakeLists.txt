cmake_minimum_required(VERSION 3.14)
project(tree_mapping_geometry)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

# ---- Find dependencies ----------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_conversions REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters segmentation features visualization)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)

# ---- Shared utility library ----------------------------------------------
add_library(${PROJECT_NAME}_utils
  src/main_UtilsAndParams.cpp
)
target_include_directories(${PROJECT_NAME}_utils
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}_utils PUBLIC ${PCL_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME}_utils PUBLIC
  rclcpp sensor_msgs pcl_conversions
)

# ---- cylinder_fitting_residual_eval (residual sanity-check binary) -------
add_executable(cylinder_fitting_residual_eval
  src/cylinderFitting_ResidualEval.cpp
)
target_link_libraries(cylinder_fitting_residual_eval
  ${PROJECT_NAME}_utils
  Ceres::ceres
  ${OpenCV_LIBS}
)
ament_target_dependencies(cylinder_fitting_residual_eval
  rclcpp sensor_msgs visualization_msgs geometry_msgs std_msgs cv_bridge pcl_conversions
)

# ---- cylinder_fitter_node (full Ceres optimizer + RViz markers) ----------
add_executable(cylinder_fitter_node
  src/cylinder_fitter_node.cpp
)
target_link_libraries(cylinder_fitter_node
  ${PROJECT_NAME}_utils
  Ceres::ceres
  ${OpenCV_LIBS}
)
ament_target_dependencies(cylinder_fitter_node
  rclcpp sensor_msgs visualization_msgs geometry_msgs std_msgs builtin_interfaces
  cv_bridge pcl_conversions
)

# ---- branch_segmenter_node (range-image based trunk/branch separation) ---
add_executable(branch_segmenter_node
  src/branch_segmenter_node.cpp
)
target_link_libraries(branch_segmenter_node
  ${PROJECT_NAME}_utils
  ${OpenCV_LIBS}
)
ament_target_dependencies(branch_segmenter_node
  rclcpp sensor_msgs std_msgs cv_bridge pcl_conversions
)

# ---- Install --------------------------------------------------------------
install(DIRECTORY include/
  DESTINATION include
)
install(TARGETS
  ${PROJECT_NAME}_utils
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(TARGETS
  cylinder_fitting_residual_eval
  cylinder_fitter_node
  branch_segmenter_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_utils)
ament_export_dependencies(rclcpp sensor_msgs pcl_conversions PCL Eigen3)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

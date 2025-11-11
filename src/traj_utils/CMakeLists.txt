cmake_minimum_required(VERSION 3.8)
project(traj_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 标准格式0
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(bspline REQUIRED)
find_package(poly_traj REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(PCL 1.7 REQUIRED)

# 标准格式1
set(dependencies
  rclcpp
  geometry_msgs
  visualization_msgs
  Eigen3
  bspline
  poly_traj
  cv_bridge
  PCL
)

# 标准格式2
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

# 标准格式3
# 改为静态库，避免需要 -fPIC
add_library(${PROJECT_NAME} STATIC src/planning_visualization.cpp)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

# 标准格式4
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

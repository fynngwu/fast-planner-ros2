cmake_minimum_required(VERSION 3.8)
project(path_searching)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(plan_env REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)


include_directories(
  include
  ${Eigen3_INCLUDE_DIRS} 
)
# Create library
add_library(${PROJECT_NAME}
  src/kinodynamic_astar.cpp
)
set(dependencies rclcpp std_msgs geometry_msgs nav_msgs visualization_msgs plan_env)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

# Install targets
install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers - with trailing slash to copy contents directly to include/
# This creates install/include/path_searching/xxx.h structure, allowing #include <path_searching/xxx.h>
install(DIRECTORY include/
  DESTINATION include/
)

# Export include directories explicitly
# This ensures other packages can use #include <path_searching/xxx.h>
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})


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

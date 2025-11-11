cmake_minimum_required(VERSION 3.8)
project(plan_env)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

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

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

# Library: EDT Environment
add_library(${PROJECT_NAME} SHARED
  src/edt_environment.cpp
  src/obj_predictor.cpp
  src/sdf_node.cpp
)
set(dependencies rclcpp nav_msgs std_msgs sensor_msgs visualization_msgs geometry_msgs)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})




# Executable: SDF Node
add_executable(sdf_node
  src/main.cpp
  src/sdf_node.cpp
)

ament_target_dependencies(sdf_node
  ${dependencies}
)


# Executable: Odom Publisher (for testing)
add_executable(odom_publisher
  src/main_odom_publisher.cpp
)

ament_target_dependencies(odom_publisher
  rclcpp
  nav_msgs
)

# Install targets
# Libraries should be installed to lib/ with EXPORT and INCLUDES DESTINATION
# This allows other packages to use ament_target_dependencies to find headers
install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

# Executables go to lib/${PROJECT_NAME}
install(TARGETS sdf_node odom_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# Install config files - without trailing slash to preserve directory structure
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)


install(DIRECTORY include/
  DESTINATION include
)

# Export include directories explicitly
# This ensures other packages can use #include <plan_env/xxx.h>
# The install(DIRECTORY include/) above creates install/include/plan_env/ structure
# and ament_export_include_directories makes install/include available to consumers
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  ${dependencies}
)
# ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()

cmake_minimum_required(VERSION 3.8)
project(plan_manage)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 标准格式0
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(plan_env REQUIRED)
find_package(path_searching REQUIRED)
find_package(bspline REQUIRED)
find_package(bspline_opt REQUIRED)
find_package(traj_utils REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(ament_cmake_python REQUIRED)

# 自定义消息定义
set(MSG_FILES
  "msg/Bspline.msg"
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  DEPENDENCIES geometry_msgs builtin_interfaces
)

# 标准格式1
set(dependencies
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_ros
  plan_env
  path_searching
  bspline
  bspline_opt
  traj_utils
  rosidl_default_runtime
)

# 标准格式2
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)

# 获取消息类型支持目标
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# 标准格式3
# 创建可执行文件（像原始 Fast-Planner 一样，直接编译所有源文件）
add_executable(fast_planner_node
  src/fast_planner_node.cpp
  src/kino_replan_fsm.cpp
  src/planner_manager.cpp
)
target_compile_features(fast_planner_node PUBLIC cxx_std_17)
ament_target_dependencies(fast_planner_node ${dependencies})
target_link_libraries(fast_planner_node ${PCL_LIBRARIES})
target_link_libraries(fast_planner_node /usr/local/lib/libnlopt.so)
target_link_libraries(fast_planner_node ${cpp_typesupport_target})
add_dependencies(fast_planner_node ${cpp_typesupport_target})

add_executable(traj_server_node src/traj_server.cpp)
target_compile_features(traj_server_node PUBLIC cxx_std_17)
ament_target_dependencies(traj_server_node ${dependencies})
target_link_libraries(traj_server_node ${cpp_typesupport_target})
add_dependencies(traj_server_node ${cpp_typesupport_target})

# 安装目标
install(TARGETS
  fast_planner_node
  traj_server_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# 安装头文件、配置、启动文件和消息
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
# 只安装 Python launch 文件，排除旧的 ROS1 XML 文件
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.launch.py"
)
install(DIRECTORY msg DESTINATION share/${PROJECT_NAME})

# 安装 Python 脚本
install(PROGRAMS scripts/odom_simulator.py DESTINATION lib/${PROJECT_NAME})

# 标准格式4
ament_export_include_directories(include)
ament_export_dependencies(${dependencies} rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

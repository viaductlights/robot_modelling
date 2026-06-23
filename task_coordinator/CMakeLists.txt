cmake_minimum_required(VERSION 3.8)
project(task_coordinator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rosidl_default_generators REQUIRED)

include_directories(include ${EIGEN3_INCLUDE_DIRS})
ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(Eigen3)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SelectPose.srv"
  "srv/MoveToPose.srv"
  "srv/JogStep.srv"
  DEPENDENCIES geometry_msgs
)

add_executable(sim_task src/sim_task.cpp)
ament_target_dependencies(sim_task rclcpp moveit_ros_planning_interface geometry_msgs tf2 tf2_ros tf2_geometry_msgs Eigen3)

add_executable(hmi_motion_server src/hmi_motion_server.cpp src/arm_setup.cpp)
ament_target_dependencies(hmi_motion_server rclcpp std_srvs moveit_ros_planning_interface ament_index_cpp geometry_msgs)

rosidl_get_typesupport_target(task_coordinator_typesupport_cpp ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(hmi_motion_server "${task_coordinator_typesupport_cpp}")

install(TARGETS sim_task hmi_motion_server
	DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
	DESTINATION share/${PROJECT_NAME}
)

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

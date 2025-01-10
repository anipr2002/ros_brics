cmake_minimum_required(VERSION 3.8)
project(ros_brics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# C++ dependencies
include_directories(include)

# Create cpp executables
add_executable(move_program src/move_program.cpp)
add_executable(waypoint_navigator src/waypoint_navigator.cpp)
add_executable(scheduler src/scheduler.cpp)
add_executable(test_move src/test_move.cpp)

ament_target_dependencies(move_program rclcpp tf2_eigen tf2_geometry_msgs tf2_ros moveit_core moveit_ros_planning moveit_ros_planning_interface moveit_servo)
ament_target_dependencies(waypoint_navigator rclcpp tf2_eigen tf2_geometry_msgs tf2_ros moveit_core moveit_ros_planning moveit_ros_planning_interface moveit_servo)
ament_target_dependencies(scheduler rclcpp tf2_eigen tf2_geometry_msgs tf2_ros moveit_core moveit_ros_planning moveit_ros_planning_interface moveit_servo)
ament_target_dependencies(test_move rclcpp tf2_eigen tf2_geometry_msgs tf2_ros moveit_core moveit_ros_planning moveit_ros_planning_interface moveit_servo)

# Install cpp executables
install(TARGETS
    move_program
    waypoint_navigator
    scheduler
    test_move
    DESTINATION lib/${PROJECT_NAME})

# Install python modules
# ament_python_install_package(${PROJECT_NAME})

# Install python executables
install(PROGRAMS
  scripts/joint_transform_printer.py
  scripts/point_recording.py
  scripts/tf_listener.py
  scripts/yolo_node.py
  scripts/yolo_subscriber.py
  scripts/robot_state.py
  scripts/test_movepy.py
  scripts/save_transform.py
  scripts/aruco_publisher.py
  scripts/yolov8_obb_publisher.py
  scripts/yolov8_obb_subscriber.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Generate ROS 2 interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Yolo.msg"
  "msg/RobotState.msg"
  "msg/InferenceResult.msg"
  "msg/Yolov8Inference.msg"
  "srv/ProcessBrick.srv"
  DEPENDENCIES geometry_msgs sensor_msgs
)

# Export runtime dependencies
ament_export_dependencies(rosidl_default_runtime)

ament_package()

cmake_minimum_required(VERSION 3.8)
project(ros_brics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(robotiq_gripper_ros2 REQUIRED)
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
find_package(Qt5 REQUIRED COMPONENTS Widgets)

# C++ dependencies
include_directories(include)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp)

# Create cpp executables
add_executable(move_program src/move_program.cpp)
add_executable(waypoint_navigator src/waypoint_navigator.cpp)
add_executable(scheduler src/scheduler.cpp)
add_executable(test_move src/test_move.cpp)
add_executable(move_program_gui src/move_program_gui.cpp)
add_executable(move_program_listener src/move_program_listener.cpp)
add_executable(move_to_pose_server src/move_to_pose_server.cpp)

ament_target_dependencies(move_to_pose_server rclcpp rclcpp_action geometry_msgs moveit_ros_planning_interface tf2 tf2_geometry_msgs)
ament_target_dependencies(move_program_gui rclcpp moveit_ros_planning_interface tf2_geometry_msgs)
ament_target_dependencies(move_program_listener rclcpp geometry_msgs moveit_ros_planning_interface tf2 tf2_geometry_msgs)
ament_target_dependencies(move_program rclcpp tf2_eigen tf2_geometry_msgs tf2_ros moveit_core moveit_ros_planning moveit_ros_planning_interface moveit_servo)
ament_target_dependencies(waypoint_navigator rclcpp tf2_eigen tf2_geometry_msgs tf2_ros moveit_core moveit_ros_planning moveit_ros_planning_interface moveit_servo)
ament_target_dependencies(scheduler rclcpp tf2_eigen tf2_geometry_msgs tf2_ros moveit_core moveit_ros_planning moveit_ros_planning_interface moveit_servo)
ament_target_dependencies(test_move rclcpp tf2_eigen tf2_geometry_msgs tf2_ros moveit_core moveit_ros_planning moveit_ros_planning_interface moveit_servo)

target_link_libraries(move_program_gui Qt5::Widgets)
target_link_libraries(move_program_listener Qt5::Widgets)




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
  scripts/printer.py
  scripts/camera_publisher.py
  scripts/pipeline.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Generate ROS 2 interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DetectionResult.msg"
  "msg/RobotState.msg"
  "msg/InferenceResult.msg"
  "msg/Yolov8Inference.msg"
  "srv/ProcessBrick.srv"
  "action/MoveToPose.action"
  DEPENDENCIES geometry_msgs sensor_msgs
)

rosidl_target_interfaces(move_to_pose_server
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

# Install cpp executables
install(TARGETS
    move_program
    waypoint_navigator
    scheduler
    test_move
    move_program_gui
    move_program_listener
    move_to_pose_server
    DESTINATION lib/${PROJECT_NAME}
)

# Export runtime dependencies
ament_export_dependencies(rosidl_default_runtime)

ament_package()

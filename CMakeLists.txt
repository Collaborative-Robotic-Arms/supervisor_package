cmake_minimum_required(VERSION 3.10)
project(supervisor_package)

# 1. Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# 2. Generate the Actions (.action files)
# This automatically handles the Python and C++ bindings

rosidl_generate_interfaces(${PROJECT_NAME}
"msg/SuperBrick.msg"  
"action/MoveToPose.action"
  "action/AlignToTarget.action"
  "action/ExecuteTask.action"
  "srv/GetAssemblyPlan.srv"   
  "srv/DetectBricks.srv"      
  DEPENDENCIES geometry_msgs
)


install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)


# Install the main supervisor
install(PROGRAMS
  supervisor_package/supervisor.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME supervisor)

install(PROGRAMS
  supervisor_package/supervisor2.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME supervisor_2)
  
install(PROGRAMS
  supervisor_package/supervisor_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME supervisor_node)
  
install(PROGRAMS
  supervisor_package/ar4_controller_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME ar4_controller_node
)

install(PROGRAMS
  supervisor_package/abb_controller_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME abb_controller_node
)

install(PROGRAMS
  supervisor_package/hybrid_supervisor_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME hybrid_supervisor_node)

install(PROGRAMS
  supervisor_package/mock_system.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME mock_system
)

install(PROGRAMS
  supervisor_package/mock_ar4.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME mock_ar4
)

install(PROGRAMS
  supervisor_package/mock_detection_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME mock_detection_node)

install(PROGRAMS
  supervisor_package/mock_gui_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME mock_gui_node)

install(PROGRAMS
  supervisor_package/mock_grasping_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME mock_grasping_node)

# 4. Export and Package
ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(tf2_ros tf2_geometry_msgs)
ament_package()
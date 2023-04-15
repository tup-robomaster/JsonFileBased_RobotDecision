cmake_minimum_required(VERSION 3.8)
project(robot_decision)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_lifecycle_manager REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(global_interface REQUIRED)
find_package(message_filters REQUIRED)
find_package(std_msgs REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


include_directories(
  include
)

set(node_plugins "")
  
add_library(${PROJECT_NAME} SHARED
  src/Json/jsoncpp.cpp 
  src/${PROJECT_NAME}/RobotDecision.cpp
  src/${PROJECT_NAME}_node.cpp
)

set(dependencies
  rclcpp
  rclcpp_components
  nav2_util
  nav2_lifecycle_manager
  nav2_msgs
  nav_msgs
  std_msgs
  geometry_msgs
  global_interface
  rclcpp_lifecycle 
  message_filters 
  robot_state_publisher 
  nav2_behavior_tree
  rosidl_default_generators
  tf2_ros
  tf2
  tf2_geometry_msgs
  OpenCV
)

ament_target_dependencies(
  ${PROJECT_NAME}
  ${dependencies}
)

add_executable(${PROJECT_NAME}_node
src/${PROJECT_NAME}_node.cpp
src/robot_decision/RobotDecision.cpp
src/Json/jsoncpp.cpp
)

target_link_libraries(${PROJECT_NAME}_node 
  ${PROJECT_NAME}
)

target_compile_definitions(${PROJECT_NAME}
  PRIVATE "COMPOSITION_BUILDING_DLL"
)

rclcpp_components_register_nodes(${PROJECT_NAME} 
  PLUGIN "robotdecisionsystem::decision_process"
  EXECUTABLE ${PROJECT_NAME}_node  
)

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY include/
    DESTINATION include
)

install(
  DIRECTORY launch JsonFile resources
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

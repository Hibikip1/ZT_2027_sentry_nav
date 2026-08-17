cmake_minimum_required(VERSION 3.5)
project(bt)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
# find_package(boost_asio REQUIRED)
find_package(serial_interfaces)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED  )
find_package(map_msgs REQUIRED  )
include_directories(
  include
)

add_executable(decision_node
  src/refree_subscriber_node.cpp
)

add_executable(refree_pub
  src/refree_pub_test.cpp
)

add_executable(radar_pub
  src/radar_pub_test.cpp)

add_executable(zone_manager
  src/zone_manager.cpp
)
target_link_libraries(decision_node
${YAML_CPP_LIBRARIES}

)
target_link_libraries(zone_manager
${YAML_CPP_LIBRARIES}

)

ament_target_dependencies(decision_node
  rclcpp
  behaviortree_cpp
  serial_interfaces
  tf2_geometry_msgs
  nav2_msgs
  rclcpp_action
  nav2_common
  nlohmann_json
  interfaces
  nav_msgs
  visualization_msgs
  tf2_ros
  OpenCV
  map_msgs
)
ament_target_dependencies(refree_pub
  rclcpp
  behaviortree_cpp
  interfaces
  serial_interfaces
  tf2_geometry_msgs
  nav2_msgs
  rclcpp_action
  nav2_common
  nlohmann_json
)

ament_target_dependencies(radar_pub
  rclcpp
  behaviortree_cpp
  interfaces
  serial_interfaces
  tf2_geometry_msgs
  nav2_msgs
  geometry_msgs
  rclcpp_action
  nav2_common
  nlohmann_json
)

ament_target_dependencies(zone_manager
  rclcpp
  interfaces
  visualization_msgs
  nav_msgs
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
)

install(TARGETS
  decision_node
  refree_pub
  radar_pub
  zone_manager
  DESTINATION lib/${PROJECT_NAME}
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

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/
)


ament_package()

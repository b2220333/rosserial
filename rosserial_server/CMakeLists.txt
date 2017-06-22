cmake_minimum_required(VERSION 3.5)
project(rosserial_server)

# find_package(catkin REQUIRED COMPONENTS roscpp rosserial_msgs topic_tools)
find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake_ros REQUIRED)
set(required_deps
  "rclcpp"
  "rmw_implementation"
  "std_msgs"
  "rosserial_msgs"
)

ament_auto_find_build_dependencies(REQUIRED ${required_deps})

find_package(Boost REQUIRED COMPONENTS system thread)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

# catkin_package(
#   INCLUDE_DIRS include
#   CATKIN_DEPENDS roscpp rosserial_msgs topic_tools
# )

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${rmw_implementation_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${rosserial_msgs_INCLUDE_DIRS}
)

set(libs
  ${Boost_LIBRARIES}
  ${rclcpp_LIBRARIES}
  ${rmw_implementation_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${rosserial_msgs_LIBRARIES}
)

ament_auto_add_executable(${PROJECT_NAME}_serial_node src/serial_node.cpp)
ament_target_dependencies(${PROJECT_NAME}_serial_node)

ament_auto_add_executable(${PROJECT_NAME}_socket_node src/socket_node.cpp)
ament_target_dependencies(${PROJECT_NAME}_socket_node)

ament_auto_add_executable(${PROJECT_NAME}_udp_socket_node src/udp_socket_node.cpp)
ament_target_dependencies(${PROJECT_NAME}_udp_socket_node)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include
)

ament_auto_package()

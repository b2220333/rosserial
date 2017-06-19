cmake_minimum_required(VERSION 3.5)
project(rosserial_msgs)

find_package(rosidl_default_generators REQUIRED)

set(msg_files
  msg/Log.msg
  msg/TopicInfo.msg
)

set(srv_files
  srv/RequestParam.srv
  srv/RequestMessageInfo.srv
  srv/RequestServiceInfo.srv
)

rosidl_generate_interfaces(${PROJECT_NAME} ${msg_files} ${srv_files})
ament_export_dependencies(rosidl_default_runtime)
ament_package()


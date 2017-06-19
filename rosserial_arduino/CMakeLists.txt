cmake_minimum_required(VERSION 3.5)
project(rosserial_arduino)

find_package(rosidl_default_generators REQUIRED)

set(msg_files
  msg/Adc.msg
)

set(srv_files
  srv/Test.srv
)

rosidl_generate_interfaces(${PROJECT_NAME} ${msg_files} ${srv_files})

ament_export_dependencies(rosidl_default_runtime)

ament_package()

# TODO(allenh1): consider moving the ros_lib into
#  an include, as opposed to putting it where it
#  was placed originally.
install(DIRECTORY src/ros_lib
  DESTINATION share/${PROJECT_NAME}/src
)

install(DIRECTORY arduino-cmake
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS src/rosserial_arduino/make_libraries.py
  DESTINATION share/${PROJECT_NAME}
)

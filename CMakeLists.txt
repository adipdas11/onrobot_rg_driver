cmake_minimum_required(VERSION 3.8)
project(onrobot_rg_driver)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED) 

set(srv_files
  "srv/Open.srv"
  "srv/Close.srv"
  "srv/MoveToWidth.srv"
  "srv/Stop.srv"
  "srv/SetForce.srv"
  "srv/SetWidth.srv"
  "srv/GetStatus.srv"
)

set(msg_files
  "msg/RGStatus.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
  ${msg_files}
  DEPENDENCIES builtin_interfaces
)

# install python pkg & script (as you already had)
if(NOT TARGET ament_cmake_python_copy_${PROJECT_NAME})
  ament_python_install_package(${PROJECT_NAME})
endif()

install(PROGRAMS
  onrobot_rg_driver/driver_node.py
  onrobot_rg_driver/onrobot.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY resource DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()

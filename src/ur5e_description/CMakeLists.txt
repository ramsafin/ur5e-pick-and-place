cmake_minimum_required(VERSION 3.22)
project(ur5e_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config urdf DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/generate_urdf.sh DESTINATION share/${PROJECT_NAME}/scripts)

ament_package()

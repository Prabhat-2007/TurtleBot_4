cmake_minimum_required(VERSION 3.8)
project(tb4_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch worlds rviz config DESTINATION share/${PROJECT_NAME})

ament_package()

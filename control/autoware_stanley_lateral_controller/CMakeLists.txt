cmake_minimum_required(VERSION 3.14)
project(autoware_stanley_lateral_controller)

find_package(autoware_cmake REQUIRED)
autoware_package()

set(STANLEY_LAT_CON_LIB ${PROJECT_NAME}_lib)

ament_auto_add_library(${STANLEY_LAT_CON_LIB} SHARED
  src/stanley_lateral_controller.cpp
)

ament_auto_package(
  INSTALL_TO_SHARE
  param
)
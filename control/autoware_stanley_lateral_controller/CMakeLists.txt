cmake_minimum_required(VERSION 3.14)
project(autoware_stanley_lateral_controller)

find_package(autoware_cmake REQUIRED)
autoware_package()

set(STANLEY_LAT_CON_LIB ${PROJECT_NAME}_lib)

ament_auto_add_library(${STANLEY_LAT_CON_LIB} SHARED
  src/stanley_lateral_controller.cpp
  src/stanley_utils.cpp
  src/stanley.cpp
)

ament_auto_package(
  INSTALL_TO_SHARE
  param
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_stanley_utils
    test/test_stanley_utils.cpp
  )

  target_link_libraries(test_stanley_utils
    ${STANLEY_LAT_CON_LIB}
  )
endif()
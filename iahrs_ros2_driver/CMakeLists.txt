cmake_minimum_required(VERSION 3.8)
project(iahrs_ros2_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(iahrs_ros2_driver_msgs REQUIRED)

add_library(serial src/serial.cpp)
add_library(iahrs_driver src/iahrs_driver.cpp)
add_executable(${PROJECT_NAME}_node src/iahrs_ros2_driver_node.cpp)

target_compile_features(serial PUBLIC cxx_std_17)
target_compile_features(iahrs_driver PUBLIC cxx_std_17)
target_compile_features(${PROJECT_NAME}_node PUBLIC cxx_std_17)

target_include_directories(serial PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(iahrs_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(${PROJECT_NAME}_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  sensor_msgs
  tf2_ros
  iahrs_ros2_driver_msgs
)

target_link_libraries(iahrs_driver serial)
target_link_libraries(${PROJECT_NAME}_node iahrs_driver)

install(TARGETS
  serial
  iahrs_driver
  EXPORT ${PROJECT_NAME}_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)
install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY
  launch param
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_sync_parser test/test_sync_parser.cpp)
  target_link_libraries(test_sync_parser iahrs_driver)

  ament_add_gtest(test_serial test/test_serial.cpp)
  target_link_libraries(test_serial serial util)
endif()

ament_export_targets(${PROJECT_NAME}_targets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_package()

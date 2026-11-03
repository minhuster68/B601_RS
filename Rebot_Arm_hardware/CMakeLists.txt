cmake_minimum_required(VERSION 3.8)
project(rebot_arm_rs_hardware)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/socket_can.cpp src/robstride_protocol.cpp src/rs_system.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME} hardware_interface pluginlib rclcpp rclcpp_lifecycle)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
pluginlib_export_plugin_description_file(hardware_interface hardware_plugins.xml)

add_executable(can_probe src/can_probe.cpp)
target_link_libraries(can_probe ${PROJECT_NAME})
install(TARGETS can_probe DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/motor_calibrate.py DESTINATION lib/${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)
install(FILES README.md PROTOCOL.md CALIBRATION.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(driver_test test/test_driver.cpp)
  target_link_libraries(driver_test ${PROJECT_NAME})
  ament_target_dependencies(driver_test hardware_interface pluginlib rclcpp rclcpp_lifecycle)
  ament_add_pytest_test(hardware_launch test/test_launch.py)
  ament_add_pytest_test(calibration test/test_calibration.py)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle)
ament_package()

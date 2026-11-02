cmake_minimum_required(VERSION 3.8)
project(rebot_arm_moveit)
find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)
install(PROGRAMS scripts/smoke_test.py DESTINATION lib/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(config test/test_config.py)
endif()

ament_package()

cmake_minimum_required(VERSION 3.8)
project(rebot_arm_rs_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch meshes mjcf rviz urdf
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

# Reuse the repository's multi-material RViz conversion without altering the
# portable URDF used by other model loaders.
install(FILES ../tools/rviz_urdf_compat.py
  DESTINATION share/${PROJECT_NAME}/tools
)
install(FILES README.md ../../LICENSE
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

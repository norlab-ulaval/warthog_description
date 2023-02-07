cmake_minimum_required(VERSION 3.0.2)
project(warthog_description)

find_package(ament_cmake REQUIRED COMPONENTS roslaunch)

install(DIRECTORY meshes launch urdf
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/env_run
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
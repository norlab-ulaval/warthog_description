cmake_minimum_required(VERSION 3.8)
project(warthog_description)

find_package(ament_cmake REQUIRED COMPONENTS roslaunch)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)

# Install
foreach(dir launch meshes urdf scripts)
    install(DIRECTORY ${dir}
            DESTINATION share/${PROJECT_NAME})
endforeach(dir)

ament_package()
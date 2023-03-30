cmake_minimum_required(VERSION 3.8)
project(warthog_description)

set(CMAKE_BUILD_TYPE "Release")

find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)

# Install
foreach(dir launch meshes urdf scripts)
    install(DIRECTORY ${dir}
            DESTINATION share/${PROJECT_NAME})
endforeach(dir)

ament_package()

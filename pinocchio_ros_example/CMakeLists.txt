cmake_minimum_required(VERSION 3.22)
project(pinocchio_ros_example)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(pinocchio REQUIRED)

# Build the executable
add_executable(pinocchio_example src/pinocchio_example.cpp)
ament_target_dependencies(pinocchio_example pinocchio Eigen3 eigen3_cmake_module)

ament_package()

cmake_minimum_required(VERSION 3.0)
project(pcd_reader)

find_package(PCL REQUIRED)

add_executable(pcd_reader main.cpp)
add_executable(viewer viewer.cpp)
target_link_libraries(pcd_reader ${PCL_LIBRARIES})
target_link_libraries(viewer ${PCL_LIBRARIES}) 
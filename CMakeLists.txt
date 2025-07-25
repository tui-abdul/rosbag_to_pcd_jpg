cmake_minimum_required(VERSION 3.14)
project(save_pcd_module)

find_package(pybind11 REQUIRED)
find_package(PCL REQUIRED COMPONENTS io)

pybind11_add_module(save_pcd_module save_pcd.cpp)
target_link_libraries(save_pcd_module PRIVATE ${PCL_LIBRARIES})
include_directories(${PCL_INCLUDE_DIRS})


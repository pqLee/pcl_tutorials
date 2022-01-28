cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
 
project(pcd_demo)

if(NOT DEFINED CMAKE_SUPPRESS_DEVELOPER_WARNINGS)
		    set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings")
endif()
 
find_package(PCL 1.2 REQUIRED)
 
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
 
add_executable (pcd_demo src/XXX.cpp)
target_link_libraries (pcd_demo ${PCL_LIBRARIES})

cmake_minimum_required(VERSION 3.17)
project(ICP)
find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(CMAKE_CXX_STANDARD 11)

add_executable(ICP main.cpp Plotter.cpp Plotter.h IcpManager.cpp IcpManager.h Performance.cpp Performance.h)
target_link_libraries (ICP ${PCL_LIBRARIES})
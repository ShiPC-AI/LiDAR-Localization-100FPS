#====CMake=====#
cmake_minimum_required(VERSION 3.0)
project(lidar_localization_100fps)

#======C++=====#
set(CMAKE_CXX_STANDARD 14)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
add_definitions(-std=c++14 -g)

#====When multiple PCL exists=======#
#========pcl-1.12 is selected=======#
find_package(PCL 1.12 REQUIRED)
set(PCL_INCLUDE_DIRS /usr/local/include/pcl-1.12)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#=====OpenCV=====#
find_package(OpenCV 4.2.0)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

#======Tree lib=====#
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib)
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib)

#====Subdirectory===#
add_subdirectory(base)
add_subdirectory(extractor)
add_subdirectory(localizer)
add_subdirectory(run)
add_subdirectory(map_process)
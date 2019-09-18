cmake_minimum_required(VERSION 2.8)

project(method2)
if(NOT CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "")
#set(CMAKE_BUILD_TYPE Release)
set(CMAKE_BUILD_TYPE Debug)
endif()
add_definitions(-std=c++14)

if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)


set(PCL_DIR /home/dl/Downloads/pcl-pcl-1.9.1/build/PCLConfig.cmake)
find_package(PCL 1.9 REQUIRED)

include_directories(include
    ${PCL_INCLUDE_DIRS})

add_library(segment_pallet
    src/loadpcl_file.cpp
    src/cond_removal.cpp
    src/icp_pallet.cpp
    src/cond_removal_after.cpp
#    src/momentOfInertia.cpp
    src/template_alignment.cpp
    src/extract_border.cpp
    src/planar_seg.cpp
    )

add_executable(${PROJECT_NAME} "src/main.cpp")
#add_executable(${PROJECT_NAME} "test/test_cluster_extraction.cpp")
#add_executable(${PROJECT_NAME} "test/test_seg.cpp")

target_link_libraries (${PROJECT_NAME}
    segment_pallet
    ${PCL_LIBRARIES}
    )



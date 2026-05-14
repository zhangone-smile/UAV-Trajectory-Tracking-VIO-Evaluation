cmake_minimum_required(VERSION 2.8.3)
project(stereo_vo)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        sensor_msgs
        nav_msgs
        geometry_msgs
        visualization_msgs
        tf
        message_filters
        message_generation
        camera_models
        )

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
set(STEREO_VO_OPENCV_VERSION "" CACHE STRING "Optional exact OpenCV version to require")
if(STEREO_VO_OPENCV_VERSION)
  find_package(OpenCV ${STEREO_VO_OPENCV_VERSION} EXACT REQUIRED)
else()
  find_package(OpenCV REQUIRED)
endif()

message(STATUS "Using OpenCV ${OpenCV_VERSION} from ${OpenCV_DIR}")


add_message_files(
        FILES
        relative_pose.msg
)

generate_messages(
        DEPENDENCIES
        geometry_msgs std_msgs
)

set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs nav_msgs geometry_msgs message_runtime camera_models
)


include_directories(${PROJECT_SOURCE_DIR}/include)

include_directories(
    include
    SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -O3 -Wall -pthread") # -Wextra -Werror
set(CMAKE_BUILD_TYPE "RELEASE")

add_executable(${PROJECT_NAME}
        src/stereo_vo_node.cpp
        src/parameters.cpp
        src/estimator.cpp
        )
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
                ${catkin_LIBRARIES}
                ${OpenCV_LIBS}
                ${PCL_LIBRARY}
                -ldw
        )

cmake_minimum_required(VERSION 2.8.3)
project(data_exporter)

find_package(catkin REQUIRED COMPONENTS
        cv_bridge
        pcl_conversions
        pcl_ros
        roscpp
        sensor_msgs
        )

find_package(OpenCV REQUIRED)
find_package(Qt5Core REQUIRED)

catkin_package(
        DEPENDS PCL                                #Non-catkin CMake projects
)

set(CMAKE_CXX_FLAGS "-std=c++11 -O2 -Wall ${CMAKE_CXX_FLAGS}")

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
)

add_executable(image_pointcloud_exporter
        image_pointcloud_exporter/src/image_pointcloud_exporter_node.cpp)

target_link_libraries(image_pointcloud_exporter PRIVATE
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${OpenCV_LIBRARIES}
        )

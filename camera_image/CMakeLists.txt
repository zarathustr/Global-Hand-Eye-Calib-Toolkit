cmake_minimum_required(VERSION 3.0.2)
project(camera_image)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
        image_transport
        roscpp
        sensor_msgs
        std_msgs
        cv_bridge
        tf
        )

find_package(OpenCV 3.4.5 REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES camera_image
        #  CATKIN_DEPENDS image_transport roscpp sensor_msgs std_msgs
        #  DEPENDS system_lib
)


include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)

add_executable(test_node src/main_node.cpp
        src/image_process.cpp)
target_link_libraries(test_node
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        )
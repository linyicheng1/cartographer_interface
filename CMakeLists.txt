cmake_minimum_required(VERSION 2.8)

project(pure_localization)

set(PACKAGE_DEPENDENCIES
        eigen_conversions
        message_runtime
        roscpp
        roslib
        rviz
)

find_package(Boost REQUIRED COMPONENTS iostreams)
find_package(Ceres REQUIRED COMPONENTS SuiteSparse)
find_package(Eigen3 REQUIRED)
find_package(Protobuf 3.0.0 REQUIRED)

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})

include_directories(
        /usr/local/include
        /usr/include/lua5.2
        /usr/include/cairo
        ${EIGEN3_INCLUDE_DIR}
        ${Boost_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
)

add_library(
        lib_localization
        localization.cpp
        ros_msg.cpp
)

add_executable(
        pure_localization
        test.cpp
)

target_link_libraries(
        pure_localization
        lib_localization
        /usr/local/lib/libcartographer.a
        -lglog
        -lpthread
        -lboost_system
        /usr/lib/x86_64-linux-gnu/liblua5.2.so
        /usr/lib/x86_64-linux-gnu/libcairo.so
        ${PROTOBUF_LIBRARY}
        ${CERES_LIBRARIES}
        ${Boost_LIBRARIES}
        ${catkin_LIBRARIES}
)
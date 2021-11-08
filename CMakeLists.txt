project(cartographer_interface)

cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 14)

set(PACKAGE_DEPENDENCIES
        eigen_conversions
        message_runtime
        roscpp
        roslib
        rviz
        )

# 依赖库 
# 1 cartographer
# 2 Boost 
# 3 OpenCV
find_package(cartographer REQUIRED)
find_package(Boost REQUIRED COMPONENTS iostreams)
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})

# 包含 CARTOGRAPHER 所需要的cmake文件
include("${CARTOGRAPHER_CMAKE_DIR}/functions.cmake")
option(BUILD_GRPC "build features that require Cartographer gRPC support" false)
#google_initialize_cartographer_project()

# 包含的文件

set(MAP_TEST_SRC
        cartographer_interface.cpp
        main.cpp
        sensor_ros.cpp
)
#file(GLOB MAP_TEST_SRC "*.cpp")
# gcc编译选项
add_compile_options(-g)
add_compile_options(-w)

include_directories(
        ${INCLUDE_CARTO}
        ${Boost_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
)

add_executable(cartographer_interface ${MAP_TEST_SRC})

target_link_libraries(cartographer_interface  ${PROTOBUF_LIBRARY} ${Boost_LIBRARIES} cartographer ${OpenCV_LIBS} ${catkin_LIBRARIES})
#add_dependencies(cartographer_interface ${${PROJECT_NAME}_EXPORTED_TARGETS}  cartographer)
#get_target_property(INCLUDE_CARTO cartographer INTERFACE_INCLUDE_DIRECTORIES)
#target_include_directories(cartographer_interface PUBLIC ${INCLUDE_CARTO}  ${Boost_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})


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

find_package(cartographer REQUIRED)
find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})
find_package(Boost REQUIRED COMPONENTS iostreams)

include("${CARTOGRAPHER_CMAKE_DIR}/functions.cmake")
option(BUILD_GRPC "build features that require Cartographer gRPC support" false)
google_initialize_cartographer_project()

add_compile_options(-w)

file(GLOB MAP_TEST_SRC "*.cpp")
add_compile_options(-g)

add_executable(cartographer_interface ${MAP_TEST_SRC})

target_link_libraries(cartographer_interface ${catkin_LIBRARIES} ${PROTOBUF_LIBRARY} ${Boost_LIBRARIES} cartographer)
add_dependencies(cartographer_interface ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} cartographer)
get_target_property(INCLUDE_CARTO cartographer INTERFACE_INCLUDE_DIRECTORIES)
target_include_directories(cartographer_interface PUBLIC ${INCLUDE_CARTO} ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})


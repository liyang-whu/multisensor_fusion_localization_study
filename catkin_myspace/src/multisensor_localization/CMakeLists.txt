#定义需求版本和工程名称
cmake_minimum_required(VERSION 3.0.2)
project(multisensor_localization)

#cmake相关的一些设定
SET(CMAKE_BUILD_TYPE "Release")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
add_compile_options(-std=c++11)
add_definitions(-std=c++11)

#ROS设置
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package()

#第三方库设置
set(THIRD_PART_LIBRARIES "")
include(cmake/pcl.cmake)

#自定义库设置
file(GLOB_RECURSE ALL_SRCS "*.cpp")
file(GLOB_RECURSE NODE_SRCS "src/*_node.cpp")
list(REMOVE_ITEM ALL_SRCS ${NODE_SRCS})


#链接编译
add_executable(test_node src/test_node.cpp ${ALL_SRCS} )
target_link_libraries(test_node ${catkin_LIBRARIES} ${THIRD_PART_LIBRARIES})
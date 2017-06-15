cmake_minimum_required(VERSION 2.8.3)
project(cuarl_util)
find_package(catkin REQUIRED roscpp std_msgs geometry_msgs)
catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(publish_fromfile src/publish_fromfile.cpp)
target_link_libraries(publish_fromfile ${catkin_LIBRARIES} yaml-cpp)

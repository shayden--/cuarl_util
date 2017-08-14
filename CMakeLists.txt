cmake_minimum_required(VERSION 2.8.3)
project(cuarl_util)
find_package(catkin REQUIRED roscpp rosbag std_msgs geometry_msgs aris_msgs)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS}
  ${GTEST_INCLUDE_DIRS}
)

add_executable(publish_fromfile src/publish_fromfile.cpp src/pff_run_functions.cpp)
target_link_libraries(publish_fromfile ${catkin_LIBRARIES} yaml-cpp)

add_executable(extract_to_csv src/extract_to_csv.cpp)
target_link_libraries(extract_to_csv ${catkin_LIBRARIES})

if (CATKIN_ENABLE_TESTING)
  find_package(GTest REQUIRED)
  catkin_add_gtest(utest_pff_run_functions test/utest_pff_run_functions.cpp src/pff_run_functions.cpp)
  target_link_libraries(utest_pff_run_functions ${catkin_LIBRARIES} yaml-cpp)
endif() 

cmake_minimum_required(VERSION 2.8.3)
project(uav_data_get)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cmake_modules
)

find_package(OpenCV REQUIRED)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES fly_shape
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)


include_directories(include  ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})


#set the default path for built executables to the "bin" directory
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/./bin)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/bin)


## Declare a cpp library
# add_library(flyline
#   src/fly_line.cpp
# )

 add_executable(position_data src/position_data.cpp)
 add_dependencies(position_data uav_data_get_generate_messages_cpp)
 target_link_libraries(position_data  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

 
#set optimization level 
set(CMAKE_BUILD_TYPE Release)


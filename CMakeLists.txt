cmake_minimum_required(VERSION 2.8.3)
project(dwmodule)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

add_message_files(
	FILES
	distances.msg
)

generate_messages(
	DEPENDENCIES
	std_msgs
)



catkin_package(

)


include_directories(

  ${catkin_INCLUDE_DIRS}
)


add_definitions(-std=c++0x -g -O3)

add_executable (HW src/main.cpp)

target_link_libraries(HW ${catkin_LIBRARIES})
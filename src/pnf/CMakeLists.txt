cmake_minimum_required(VERSION 2.8.3)
project(pnf)
#add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs	
)


catkin_package(INCLUDE_DIRS include
  )


include_directories(${catkin_INCLUDE_DIRS})
add_executable(pnf src/pnf.cpp)
target_link_libraries(pnf)

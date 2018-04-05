cmake_minimum_required(VERSION 2.8.3)
project(obstacle_detection)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  wizo_msgs
)

 catkin_python_setup()

catkin_package(
   CATKIN_DEPENDS std_msgs wizo_msgs
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

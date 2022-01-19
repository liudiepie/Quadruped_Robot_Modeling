cmake_minimum_required(VERSION 2.8.3)

project(testudog)

## Find ROS related packages/libraries
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg sensor_msgs)

catkin_package()

find_package(roslaunch)
find_package(Eigen3 REQUIRED)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

## Build node
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(gazebo_controller controller/src/main.cpp controller/src/testudog.cpp)
target_link_libraries(gazebo_controller Eigen3::Eigen ${catkin_LIBRARIES})




cmake_minimum_required(VERSION 2.8.3)
project(fusion)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
		roscpp
		rospy
		std_msgs
		sensor_msgs
		eigen_conversions
		cv_bridge
		image_transport
		)

list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
find_package(Eigen3 REQUIRED)
find_package (GeographicLib REQUIRED)
find_package(OpenCV 3.3 REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
		#  INCLUDE_DIRS include
		#  LIBRARIES fusion
		#  CATKIN_DEPENDS roscpp rospy std_msgs
		#  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
		include
		include/fusion
		${catkin_INCLUDE_DIRS}
#		${EIGEN3_INCLUDE_DIR}
		${OpenCV_INCLUDE_DIRS}
		ThirdParty/eigen3
		ThirdParty/sophus
		# ThirdParty/g2o
)

## Declare a C++ library
add_library(${PROJECT_NAME}
		src/fusion_node.cpp
		src/kalman_filter.cpp
		)

## Declare a C++ executable
add_executable(${PROJECT_NAME}_node app/fusion.cpp)

## Specify libraries to link a library or executable target against
set_target_properties(${PROJECT_NAME}_node PROPERTIES COMPILE_DEFINITIONS EIGEN_NO_MALLOC)
target_link_libraries(${PROJECT_NAME}_node
		${PROJECT_NAME}
		${catkin_LIBRARIES}
		${GeographicLib_LIBRARIES}
		)

###############################  read image  #######################################
add_executable(image_node app/read_image.cpp)
target_link_libraries(image_node ${catkin_LIBRARIES} ${OpenCV_LIBS})

###############################  read sensor  #######################################
add_executable(sensor_node app/read_sensor.cpp)
target_link_libraries(sensor_node ${catkin_LIBRARIES})
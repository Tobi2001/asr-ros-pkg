cmake_minimum_required(VERSION 2.4.6)
project(active_scene_recognition_psm)

add_definitions(-Wall -std=gnu++0x -ggdb)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  ism
  pbd_msgs
  std_msgs
  roscpp
  sensor_msgs
  std_srvs
  tf
  ptu_controller
  psm
)

find_package(Boost REQUIRED
  iostreams
)

find_package(Eigen REQUIRED)

include_directories(
  ${Eigen_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS tf
		  roscpp
		  pbd_msgs
		  std_msgs
		  std_srvs
		  sensor_msgs
		  geometry_msgs
		  ism
		  ptu_controller
		  psm
  DEPENDS Eigen Boost
)

############################################################
# Build active scene recognition app.
############################################################

add_executable(${PROJECT_NAME} src/asrStateMachine.cpp
				  src/ptuCamControl.cpp
				  src/spiralSearcher.cpp
				  src/asrVisualizer.cpp
				  src/algorithm/asrSceneFinderPSM.cpp
				  src/algorithm/asrPosePredictorPSM.cpp
				  src/algorithm/asrPosePredictorISM.cpp
				  src/algorithm/asrSceneFinderISM.cpp
			    )
	  



target_link_libraries(
  ${PROJECT_NAME}
  ${Eigen_LIBRARIES}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)


cmake_minimum_required(VERSION 2.8.12)

project(front_lidar_clustering)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  std_msgs
  geometry_msgs
  pcl_conversions
  ackermann_msgs
  autoware_msgs
  cv_bridge
  jsk_recognition_msgs
  jsk_rviz_plugins
)

#set(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL REQUIRED)

catkin_package(INCLUDE_DIRS include)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}

)

add_library(
  front_lidar_clustering_lib
  src/front_lidar_clustering.cpp
)

add_dependencies(front_lidar_clustering_lib 
   ${${PROJECT_NAME}_EXPORTED_TARGETS} 
   ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(front_lidar_clustering_lib
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
)

add_executable(front_lidar_clustering_node 
src/front_lidar_clustering_node.cpp)

add_dependencies(front_lidar_clustering_node 
   ${${PROJECT_NAME}_EXPORTED_TARGETS} 
   ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(front_lidar_clustering_node
   front_lidar_clustering_lib
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
)

link_directories(
	include
  ${PCL_LIBRARY_DIRS}
)

install(
  TARGETS 
  front_lidar_clustering_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)


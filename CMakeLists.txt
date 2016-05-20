cmake_minimum_required(VERSION 2.8.3)

project(octreemap)
SET(CMAKE_CXX_FLAGS -std=c++11)
find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  rospy
  std_msgs
  octomap_ros
  octomap_msgs
)


 find_package(octomap REQUIRED)


catkin_package(
  #INCLUDE_DIRS include
  #LIBRARIES octreemap
  #CATKIN_DEPENDS pcl_ros roscpp rospy std_msgs octomap_ros octomap_msgs
  #DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)
include_directories(${OCTOMAP_INCLUDE_DIRS})



 add_executable(test_pc2tooctomap src/test_pc2tooctomap.cpp)
 add_executable(test_octomaptopc2 src/test_octomaptopc2.cpp)


add_dependencies(test_pc2tooctomap ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(test_octomaptopc2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


 target_link_libraries(test_pc2tooctomap
   ${catkin_LIBRARIES}
 )
 target_link_libraries(test_octomaptopc2   ${catkin_LIBRARIES} )
link_libraries(${OCTOMAP_LIBRARIES})



## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

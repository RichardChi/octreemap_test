# octreemap_test
# octomap ROS http://wiki.ros.org/octomap

Using OctoMap
Run

sudo apt-get install ros-fuerte-octomap
to install OctoMap as stand-alone libraries with no ROS dependencies (so the package can also be used in a non-ROS setting). For ROS groovy, run

sudo apt-get install ros-groovy-octomap
This means that you compile against OctoMap without requiring any ROS-specific build tools or catkin macros (in fact, they won't work). For convenience, the system install includes CMake config files for easily finding and configuring OctoMap in your CMakeLists.txt using the regular find_package() macro:


find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})
link_libraries(${OCTOMAP_LIBRARIES})
(Add link_directories(${OCTOMAP_LIBRARY_DIRS}) only if required - usually it's not needed).

Finally, add to you package.xml (groovy and later):

<build_depend>octomap</build_depend>
<run_depend>octomap</run_depend>

octomap_ros and octomap_msgs provide messages, wrappers and conversion methods.

see https://github.com/OctoMap

This project is test octomap in ROS, pointcloud2 to octomap and cotomap to pointcloud2



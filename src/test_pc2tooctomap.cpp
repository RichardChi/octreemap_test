/*
*
*
* 
*/
#include <ros/ros.h>
#include <stdio.h>
#include <octomap/octomap.h>
#include <octomap_ros/conversions.h>
//#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

octomap::OcTree tree( 0.05 );

static void points_callback(const sensor_msgs::PointCloud2& input)
{
	octomap::Pointcloud octo_cloud;
	octomap::pointCloud2ToOctomap(input, octo_cloud);
	octomap::point3d origin (0.01f, 0.01f, 0.02f);

	tree.updateInnerOccupancy();
	tree.insertPointCloud(octo_cloud, origin);
    	// 存储octomap
    	std::cout << "writing to spherical_scan.bt..." << std::endl;
    	tree.writeBinary( "/home/gallop/data/spherical_scan.bt" );


}


static void  timerCallback(const ros::TimerEvent& timer)
{

}



int main(int argc, char  **argv)
{
	ros::init(argc, argv, "test_pc2tooctomap");
	ros::NodeHandle nh;

	ros::Subscriber points_sub = nh.subscribe("points_raw", 100000, points_callback);
	ros::Timer timer = nh.createTimer(ros::Duration(0.1), timerCallback);

	ros::spin();

	return 0;
}
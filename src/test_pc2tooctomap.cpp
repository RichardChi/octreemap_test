/*
*
*
* 
*/
#include <ros/ros.h>
#include <stdio.h>
#include <sensor_msgs/PointCloud2.h>
#include <octomap/octomap.h>
#include <octomap_ros/conversions.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

octomap::OcTree tree( 0.05 );
ros::Publisher octomap_pub ;//= n.advertise<octomap_msgs::Octomap>("octomap", 1000);

static void points_callback(const sensor_msgs::PointCloud2& input)
{
	pcl::PointCloud<pcl::PointXYZI> tmp;
	pcl::fromROSMsg(input, tmp);
	ROS_INFO("size = %d", tmp.points.size());

	octomap::Pointcloud octo_cloud;
	octomap::pointCloud2ToOctomap(input, octo_cloud);

	octomap::point3d pose (0.01f, 0.01f, 0.02f);

	
	tree.insertPointCloud(octo_cloud, pose);
	tree.updateInnerOccupancy();


	octomap_msgs::Octomap octo_msg;
	octomap_msgs::binaryMapToMsg(tree, octo_msg);
	octo_msg.header.frame_id = 'octmap';
	octo_msg.header.stamp = ros::Time::now();	
	octomap_pub.publish(octo_msg);
    	// 存储octomap
    	/*
    	std::cout << "writing to spherical_scan.bt..." << std::endl;
    	tree.writeBinary( "/home/gallop/data/spherical_scan.bt" );
	*/


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
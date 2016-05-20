/*
*
*
* 
*/
#include <ros/ros.h>
#include <iostream>
#include <assert.h>
#include <stdio.h>
#include <octomap/octomap.h>
//#include <octomap/OcTreeLabeled.h>
#include <octomap_ros/conversions.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

static ros::Publisher octo_cloud_pub;
static void map_callback(const octomap_msgs::Octomap& input)
{	
	octomap::OcTree* octo_map =  binaryMsgToMap(input);
	octomap::point3d_list points ;
	pcl::PointXYZ point_p;
	pcl::PointCloud<pcl::PointXYZ> cloud;
	for (octomap::OcTree::leaf_iterator it = octo_map->begin(); it!=octo_map->end(); ++it)
	{
		if (octo_map->isNodeOccupied(*it))
		{
			//double size = it.getSize();
			point_p.x = (double)it.getX();
			point_p.y = (double)it.getY();
			point_p.z = (double)it.getZ();
			cloud.points.push_back(point_p);
		}
	}
	ROS_INFO("cloud size = %d",(int)cloud.points.size());

	sensor_msgs::PointCloud2 octocloud;
	pcl::toROSMsg(cloud, octocloud);
	octocloud.header.stamp = ros::Time::now();
	octocloud.header.frame_id = "octocloud";
	octo_cloud_pub.publish(octocloud);
	
	


}




int main(int argc, char  **argv)
{
	ros::init(argc, argv, "test_octomaptopc2");
	ros::NodeHandle nh;

	ros::Subscriber octomap_sub = nh.subscribe("octomap", 100000, map_callback);

	ros::spin();

	return 0;
}
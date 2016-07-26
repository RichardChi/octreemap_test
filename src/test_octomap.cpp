#include <ros/ros.h>
#include <octomap/octomap.h>
#include <octomap_ros/conversions.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

using namespace std;
using namespace octomap;
using namespace octomath;

void loca_map()
{

}

int main( int argc, char** argv )
{
	ros::init(argc, argv, "test_octomap");
		ROS_INFO("Map test");
		ros::NodeHandle nh;
		ros::Publisher octomap_pub = nh.advertise<octomap_msgs::Octomap>("octomap", 2);
		ros::Publisher g_plan_pub_ = nh.advertise<nav_msgs::Path>("global_plan", 1);

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		int n = 20;
		cloud->width = n * n;
		cloud->height = 1;
		cloud->points.resize(cloud->width * cloud->height);

		for (size_t i = 0; i < n; ++i)
		{
			for(size_t j = 0; j <  n; ++j)
			{
				cloud->points[i * n + j].x = 10.0 ;
				cloud->points[i * n + j].y = -2.5 + 0.25 * j;
				cloud->points[i * n + j].z = -2.5 + 0.25 * i;
			}
		}

		cout<<cloud->points.size()<<endl;
		octomap::point3d origin(0,0,0);
		octomap::point3d end(10,4,5);
		float radius = 10;

		OcTree* tree = new OcTree(0.5);
		octomap::Pointcloud cloud_octo;
		point3d point_on_surface = origin;
		point_on_surface.x() += radius;

		for (int i = 0; i < 10; ++i)
		{
			for (int j = -10; j < 10; ++j)
			{
				for (int n = 0; n < 5; ++n)
				{
					//tree.updateNode( octomap::point3d(i*0.5, j*0.5, n*0.5), false);
					//tree.updateNode( octomap::point3d(i*0.5 , j*0, -n*0.5), false);
				}
			}	
			//tree.insertRay(octomap::point3d(-0.25,-0.25,-0.25), octomap::point3d(-0.25+i,4.25, -0.25));
			//tree.insertRay(octomap::point3d(-0.25,-0.25,-0.25), octomap::point3d(-0.25+i,-4.25, -0.25));
			//tree.insertRay(octomap::point3d(0,0,0), octomap::point3d(i,0.0, 0));
			//tree.updateNode( octomap::point3d(i,4.0, 0), false );
			//tree.updateNode( octomap::point3d(i, -4.0, 0), false );
			//tree.updateNode( octomap::point3d(i, 4.0,0), false );
		}
			
		for (int i = 0; i<100; ++i)
		{
			for (auto p:cloud->points)
			{
				cloud_octo.push_back( p.x+i*0, p.y+i*0, p.z );
				//tree.updateNode( octomap::point3d(p.x + i * 0.05 , p.y + i*0.0 , p.z), true );
				//tree.insertRay(octomap::point3d(-0.25,-0.25,-0.25), octomap::point3d(p.x+ i* 0.1 , p.y + i*0.0 , p.z));
				//tree.updateNode( octomap::point3d(p.x + i, p.y - i * 2, p.z + i *2), true );
				//tree.insertRay(octomap::point3d(p.x + i, p.y + i * 2, p.z + i *2), octomap::point3d(0,0,0));

			}
						
			//tree->insertPointCloud( cloud_octo, octomap::point3d( 0.0, 0.0 , 0.0 ) );
				
		}
		//octomap::point3d origin(0,0,0);
		//octomap::point3d end(10,4,5);
		//tree->insertRay(origin, end);
		//end = octomap::point3d(10,-4,3);
		//tree->insertRay(origin, end);
		tree->updateNode( octomap::point3d(0,0,0), true);
		tree->updateNode( octomap::point3d(0.5,0,0), true);
		tree->updateNode( octomap::point3d(0,0.5,0), true);
		tree->updateNode( octomap::point3d(0.5,0.5,0), true);
		tree->updateNode( octomap::point3d(0,0,0.5), true);
		tree->updateNode( octomap::point3d(0.5,0,0.5), true);
		tree->updateNode( octomap::point3d(0,0.5,0.5), true);
		tree->updateNode( octomap::point3d(0.5,0.5,0.5), true);
		tree->updateInnerOccupancy();

		cout<<tree->getResolution() <<endl;
		cout<<tree->getTreeDepth() <<endl;
		cout<<tree->getNumLeafNodes ()<<endl;
		cout<<tree->size()<<endl;
		cout<<tree->calcNumNodes()<<endl;		
			  /*
		for(int i=0;i<10;++i)
		{
			octomap::OcTreeNode* octnode = tree.search(1.0+i *1,0,0);
			//cout<<"ok"<<endl;
			bool flag = tree.isNodeOccupied(octnode);
			if (flag)
			{
				cout<<i<<":"<<flag<<endl;
			}
			
		}*/
		std::vector<geometry_msgs::PoseStamped> path_test;
		geometry_msgs::PoseStamped pose;
		//octomap::point3d end(10,2,2);

/*
		std::vector<octomap::point3d> ray;
		
		if (tree->search(end))
		{
			cout<<"OK"<<endl;
			tree->computeRay(origin, end, ray);
		}
		for (int i = 0; i < ray.size(); ++i)
		{
			pose.pose.position.x = ray[i](0) ;
		 	pose.pose.position.y = ray[i](1) ;
		 	pose.pose.position.z = ray[i](2) ;
		 	path_test.push_back(pose);
		}
		pose.pose.position.x = end(0);
		pose.pose.position.y = end(1);
		pose.pose.position.z = end(2);
		path_test.push_back(pose);

		*/
		int i = 0;
		for(octomap::OcTree::iterator iter = tree->begin(); iter != tree->end(); ++iter)
		{
		 	
		 	//
			bool flag = tree->isNodeOccupied(*iter);
			//if (!flag)
			{
				pose.pose.position.x = (double)iter.getX() ;
		 		pose.pose.position.y = (double)iter.getY() ;
		 		pose.pose.position.z = (double)iter.getZ() ;
		 		path_test.push_back(pose);
		 		//tree.updateNode( iter.getCoordinate(), true);
		 		//std::cout << iter.getDepth() << " " << i <<" "<<iter.getCoordinate()<< std::endl;
		 		//cout<<iter.getkey()<<endl;
				//cout<<i<<":"<<"flag"<<":"<<iter.getSize()<<endl;
			}
			i++;
		}
		tree->updateInnerOccupancy();
		cout<<i<<endl;
		cout<<tree->begin().getCoordinate()<<endl;
		cout<<tree->begin_tree().getX()<<" "<<tree->begin_tree().getY()<<" "<<tree->begin_tree().getZ()<<endl;
		nav_msgs::Path g_path;
		g_path.poses.resize(path_test.size());
		for (size_t i=0; i<path_test.size(); ++i)
		{
			g_path.poses[i]=path_test[i];
		}
		ros::Rate rate(10);
		bool status = ros::ok();
		while(status)
		{
				octomap_msgs::Octomap octo_msg;
				octomap_msgs::binaryMapToMsg(*tree, octo_msg);
				octo_msg.header.frame_id = "/guidance";
				octo_msg.header.stamp = ros::Time::now();  
				octomap_pub.publish(octo_msg);
				
				g_path.header.frame_id = "/guidance";
				g_path.header.stamp = ros::Time::now();
				g_plan_pub_.publish(g_path);
				status = ros::ok();
				rate.sleep();
		}

		cout<<"done."<<endl;
	
		return 0;
}

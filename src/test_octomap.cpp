#include <ros/ros.h>
#include <octomap/octomap.h>
#include <octomap_ros/conversions.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>

using namespace std;

int main( int argc, char** argv )
{
	ros::init(argc, argv, "test_octomap");
    	ROS_INFO("Map test");
    	ros::NodeHandle nh;
    	ros::Publisher octomap_pub = nh.advertise<octomap_msgs::Octomap>("octomap", 2);

    	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    	int n = 10;
    	cloud->width = n * n;
    	cloud->height = 1;
    	cloud->points.resize(cloud->width * cloud->height);

    	for (size_t i = 0; i < n; ++i)
    	{
    		for(size_t j = 0; j < n; ++j)
    		{
    			cloud->points[i * n + j].x = 10.0;
    			cloud->points[i * n + j].y = -2.5 + 0.5 * j;
    			cloud->points[i * n + j].z = -2.5 + 0.5 * i;
    		}
    	}

    	cout<<cloud->points.size()<<endl;

    	octomap::OcTree tree( 0.5 );
    	octomap::Pointcloud cloud_octo;
        	
    	for (int i = 0; i<100; ++i)
    	{
    		for (auto p:cloud->points)
    		{
    			cloud_octo.push_back( p.x+i*0.1, p.y+i*0, p.z );
    			//tree.updateNode( octomap::point3d(p.x + i * 0.05 , p.y + i*0.0 , p.z), true );
    			//tree.insertRay(octomap::point3d(-0.25,-0.25,-0.25), octomap::point3d(p.x+ i* 0.1 , p.y + i*0.0 , p.z));
    			//tree.updateNode( octomap::point3d(p.x + i, p.y - i * 2, p.z + i *2), true );
    			//tree.insertRay(octomap::point3d(p.x + i, p.y + i * 2, p.z + i *2), octomap::point3d(0,0,0));

    		}
            			
            		tree.insertPointCloud( cloud_octo, octomap::point3d( 0.0 +i *0.1 , 0.0 , 0.0 ) );
              	
    	}
    	tree.updateInnerOccupancy();
    	cout<<tree.getResolution() <<endl;
    	cout<<tree.getTreeDepth() <<endl;
    	cout<<tree.getNumLeafNodes ()<<endl;

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
    	int i = 0;
              for(octomap::OcTree::leaf_iterator iter = tree.begin(); iter != tree.end(); iter++)
              {
                	bool flag = tree.isNodeOccupied(*iter);
                	//if (flag)
                	{
                    		cout<<i<<":"<<flag<<":"<<iter.getSize()<<endl;
                	}
                	i++;
              }
    	
              ros::Rate rate(10);
    	bool status = ros::ok();
    	while(status)
    	{
        		octomap_msgs::Octomap octo_msg;
        		octomap_msgs::binaryMapToMsg(tree, octo_msg);
        		octo_msg.header.frame_id = "/guidance";
        		octo_msg.header.stamp = ros::Time::now();  
        		octomap_pub.publish(octo_msg);
        		status = ros::ok();
        		rate.sleep();
    	}

    	cout<<"done."<<endl;
    
    	return 0;
}

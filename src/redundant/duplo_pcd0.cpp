#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>

#include <iostream>
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
//#include <vector>

//#include <pcl/visualization/cloud_viewer.h>

using namespace std;
using namespace pcl;

int main (int argc, char** argv)
{
  ros::init(argc, argv, "duplo_pcd");
  if (argc != 2) {
	ROS_INFO("Incorrect number of arguments. \nUsage: duplo_pcd filename.pcd");
	return (1);
  }
  
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<duplo_ros::Process_PCD>("process_pcd");
  duplo_ros::Process_PCD srv;
  srv.request.fname = argv[1];

  if (client.call(srv))
  {
	ROS_INFO("Processed the point cloud.");
  } else {
	ROS_ERROR("Failed to call service process_pcd.");
	return 1;
  }
 
  return (0);
}

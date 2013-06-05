#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "duplo_ros/Process_PCD.h"
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

int
  main (int argc, char** argv)
{
  ros::init(argc, argv, "grasp_duplo_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("grasp_duplo", grasp_and_bin);
  ROS_INFO("Ready to grasp and bin bricks.");
  ros::spin();

//  ros::ServiceClient client = n.serviceClient<duplo_ros::Grasp_Duplo>("grasp_duplo");

  return (0);
}

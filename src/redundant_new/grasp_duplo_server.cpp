#include <ros/ros.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include <sensor_msgs/point_cloud_conversion.h>

#include "duplo_v0/Grasp_Duplo.h"

ros::Publisher pub;
sensor_msgs::PointCloud2 to_pick;
//bool grasp_service_called = false;

bool grasp_and_bin(duplo_v0::Grasp_Duplo::Request &req,
                   duplo_v0::Grasp_Duplo::Response &res)
{
  ROS_INFO("Publishing the point cloud to be picked up.");
  pub.publish(req.pick_cluster);  
  return true;
}

int main(int argc, char **argv)
{
  //initialize the ROS node
  ros::init(argc, argv, "grasp_duplo_server");
  ros::NodeHandle nh;

  ROS_INFO("Entered grasp_and_bin main\n");

  //grasp_service_called = false;

  pub = nh.advertise<sensor_msgs::PointCloud2>("pick_place",1);
  ros::ServiceServer service = nh.advertiseService("grasp_duplo", grasp_and_bin);

  /*client = n.serviceClient<duplo_v0::Pick_Place>("pick_place");
  duplo_v0::Pick_Place srv;

  while (ros::ok() && grasp_service_called ==false) {
      ros::spinOnce();
      if (grasp_service_called) {
	srv.request.pick_cluster = to_pick;

        if (client.call(srv)) {
		ROS_INFO("Finished manipulation.");
        } else {
	        ROS_ERROR("Failed to call service pick_place.");
		return -1;
        }
      }	
    }
  */
  ros::spin();
  return 0;
}

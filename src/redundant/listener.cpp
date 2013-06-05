#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "duplo_notouch_onebin/Transform_Point_Cloud.h"



bool transform(duplo_notouch_onebin::Transform_Point_Cloud::Request &req,
                   duplo_notouch_onebin::Transform_Point_Cloud::Response &res)
{
  ROS_INFO("Transforming the point cloud");

  tf::TransformListener listener;
  sensor_msgs::PointCloud transformed;
  listener.transformPointCloud(req.target_frame,req.input_pcd,transformed);
  res.transformed_pcd = transformed;

  return true;
}

int main(int argc, char **argv)
{
  //initialize the ROS node
  ros::init(argc, argv, "transform_listener");
  ros::NodeHandle nh;

  ROS_INFO("Waiting to transform\n");
  ros::ServiceServer service = nh.advertiseService("transform_point_cloud",transform);

  ros::spin();
  return 0;
}
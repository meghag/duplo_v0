#include <ros/ros.h>
#include <pcl/ros/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include <sensor_msgs/point_cloud_conversion.h>

#include <tf/transform_listener.h>

#include <actionlib/client/simple_action_client.h>
#include <tabletop_object_detector/TabletopDetection.h>
#include <tabletop_collision_map_processing/TabletopCollisionMapProcessing.h>
#include <object_manipulation_msgs/PickupAction.h>
#include <object_manipulation_msgs/PlaceAction.h>
#include "duplo_notouch_onebin/Grasp_Duplo.h"
#include "duplo_notouch_onebin/Transform_Point_Cloud.h"

//ros::Publisher pub;
pcl::PointCloud<pcl::PointXYZRGB> to_pick;
bool start_manipulation = false;

float binx = 0.8;
float biny = -0.2;
float binz = 0.6;

void manipulate(const sensor_msgs::PointCloud2::ConstPtr& pcd)
{   
ROS_INFO(" Setting the flag to start manipulation.");
  //  pub.publish(req.pick_cluster);
  fromROSMsg(*pcd,to_pick);
  start_manipulation = true;
  
  //  return true;
}

int main(int argc, char **argv)
{
  //initialize the ROS node
  ros::init(argc, argv, "manipulator");
  ros::NodeHandle nh;

  ROS_INFO("Waiting to receive a point cloud to manipulate\n");

  //grasp_service_called = false;

  ros::Subscriber sub = nh.subscribe("pick_place",1,manipulate);
  //  ros::ServiceServer service = nh.advertiseService("grasp_duplo", grasp_and_bin);

  /*client = n.serviceClient<duplo_notouch_onebin::Pick_Place>("pick_place");
    duplo_notouch_onebin::Pick_Place srv;
  */

  while (ros::ok() && start_manipulation ==false) {
    ros::spinOnce();
    if (start_manipulation==true) {
      start_manipulation = false;

      //set service and action names
      const std::string OBJECT_DETECTION_SERVICE_NAME = "/object_detection";
      const std::string COLLISION_PROCESSING_SERVICE_NAME = 
	"/tabletop_collision_map_processing/tabletop_collision_map_processing";
      const std::string PICKUP_ACTION_NAME = 
	"/object_manipulator/object_manipulator_pickup";
      const std::string PLACE_ACTION_NAME = 
	"/object_manipulator/object_manipulator_place";

      //create service and action clients
      ros::ServiceClient object_detection_srv;
      ros::ServiceClient collision_processing_srv;
      actionlib::SimpleActionClient<object_manipulation_msgs::PickupAction> 
	pickup_client(PICKUP_ACTION_NAME, true);
      actionlib::SimpleActionClient<object_manipulation_msgs::PlaceAction> 
	place_client(PLACE_ACTION_NAME, true);

      //wait for detection client
      while ( !ros::service::waitForService(OBJECT_DETECTION_SERVICE_NAME, 
					    ros::Duration(2.0)) && nh.ok() ) 
	{
	  ROS_INFO("Waiting for object detection service to come up");
	}
      if (!nh.ok()) exit(0);
      object_detection_srv = 
	nh.serviceClient<tabletop_object_detector::TabletopDetection>
	(OBJECT_DETECTION_SERVICE_NAME, true);

      //wait for collision map processing client
      while ( !ros::service::waitForService(COLLISION_PROCESSING_SERVICE_NAME, 
					    ros::Duration(2.0)) && nh.ok() ) 
	{
	  ROS_INFO("Waiting for collision processing service to come up");
	}
      if (!nh.ok()) exit(0);
      collision_processing_srv = 
	nh.serviceClient
	<tabletop_collision_map_processing::TabletopCollisionMapProcessing>
	(COLLISION_PROCESSING_SERVICE_NAME, true);

      //wait for pickup client
      while(!pickup_client.waitForServer(ros::Duration(2.0)) && nh.ok())
	{
	  ROS_INFO_STREAM("Waiting for action client " << PICKUP_ACTION_NAME);
	}
      if (!nh.ok()) exit(0);  

      //wait for place client
      while(!place_client.waitForServer(ros::Duration(2.0)) && nh.ok())
	{
	  ROS_INFO_STREAM("Waiting for action client " << PLACE_ACTION_NAME);
	}
      if (!nh.ok()) exit(0);    


      //call the tabletop detection
      ROS_INFO("Calling tabletop detector");
      tabletop_object_detector::TabletopDetection detection_call;
      //we want recognized database objects returned
      //set this to false if you are using the pipeline without the database
      detection_call.request.return_clusters = true;
      //we want the individual object point clouds returned as well
      detection_call.request.return_models = true;
      if (!object_detection_srv.call(detection_call))
	{
	  ROS_ERROR("Tabletop detection service failed");
	  return false;
	}
      if (detection_call.response.detection.result != 
	  detection_call.response.detection.SUCCESS)
	{
	  ROS_ERROR("Tabletop detection returned error code %d", 
		    detection_call.response.detection.result);
	  return false;
	}
      if (detection_call.response.detection.clusters.empty() && 
	  detection_call.response.detection.models.empty() )
	{
	  ROS_ERROR("The tabletop detector detected the table, but found no objects");
	  return false;
	}

      //call collision map processing
      ROS_INFO("Calling collision map processing");
      tabletop_collision_map_processing::TabletopCollisionMapProcessing 
	processing_call;
      //pass the result of the tabletop detection 
      processing_call.request.detection_result = detection_call.response.detection;
      //ask for the exising map and collision models to be reset
      processing_call.request.reset_static_map = true;
      processing_call.request.reset_collision_models = true;
      processing_call.request.reset_attached_models = true;
      //ask for a new static collision map to be taken with the laser
      //after the new models are added to the environment
      processing_call.request.take_static_collision_map = true;
      //ask for the results to be returned in base link frame
      processing_call.request.desired_frame = "base_link";
      if (!collision_processing_srv.call(processing_call))
	{
	  ROS_ERROR("Collision map processing service failed");
	  return false;
	}
      //the collision map processor returns instances of graspable objects
      if (processing_call.response.graspable_objects.empty())
	{
	  ROS_ERROR("Collision map processing returned no graspable objects");
	  return false;
	}

      //call object pickup
      ROS_INFO("Calling the pickup action");
      object_manipulation_msgs::PickupGoal pickup_goal;
  
      //pass one of the graspable objects returned by the collission map processor

      /*********************************************************************/
      /*********************************************************************/
      /********** Changed *********/
      //pickup_goal.target = processing_call.response.graspable_objects.at(0);


	  ros::ServiceClient transform_client = 
		nh.serviceClient<duplo_notouch_onebin::Transform_Point_Cloud>("transform_point_cloud");
	  duplo_notouch_onebin::Transform_Point_Cloud srv_transform;

	  ROS_INFO("Point cloud:cloud size = %ld\n", (long int)to_pick.points.size());
  	  for (size_t i = 0; i < 10; ++i)
		ROS_INFO("\t%f\t%f\t%f\n",to_pick.points[i].x, to_pick.points[i].y, to_pick.points[i].z);

	  object_manipulation_msgs::GraspableObject object;
	  //object.reference_frame_id = "/openni_rgb_frame";
	  sensor_msgs::PointCloud goal_pcd;
	  sensor_msgs::PointCloud2 pick_cluster;
      toROSMsg(to_pick,pick_cluster);
	  
	  tf::TransformListener listener;
	  sensor_msgs::PointCloud2 transformed;
	  pick_cluster.header.frame_id = "openni_rgb_optical_frame";
	  listener.waitForTransform("openni_rgb_optical_frame","base_link",
	                            pick_cluster.header.stamp,ros::Duration(2.0));
	  pcl_ros::transformPointCloud("base_link",pick_cluster,transformed,listener);
	  //res.transformed_pcd = transformed;

	  if (sensor_msgs::convertPointCloud2ToPointCloud(transformed,goal_pcd) == true)
	  {
		ROS_INFO("Frame Id of input point cloud cluster is: %s\n", goal_pcd.header.frame_id.c_str());
	  ROS_INFO("Target frame id is: %s\n", detection_call.response.detection.table.pose.header.frame_id.c_str());
		goal_pcd.header.frame_id = "base_link";
		goal_pcd.header.stamp = ros::Time::now();
	  
		  object.cluster = goal_pcd;
		  object.reference_frame_id = "base_link";
		  pickup_goal.target = object;
		  ROS_INFO("Set the goal target as a graspable object\n");
		
	  } else {
		ROS_ERROR("Conversion from pointcloud2 to pointcloud failed.\n");
		return false;
      }
	 
  
      //pass the name that the object has in the collision environment
      //this name was also returned by the collision map processor
      //pickup_goal.collision_object_name = 
      //  processing_call.response.collision_object_names.at(0);
      /*********************************************************************/
      /*********************************************************************/

  
      //pass the collision name of the table, also returned by the collision 
      //map processor
      pickup_goal.collision_support_surface_name = 
	processing_call.response.collision_support_surface_name;

  
      /*********************************************************************/
      /*********************************************************************/
      /******* Added: allowing collisions with the table ******/
      pickup_goal.allow_gripper_support_collision = true;
      /*********************************************************************/
      /*********************************************************************/
  
      //pick up the object with the right arm
      pickup_goal.arm_name = "right_arm";
      //specify the desired distance between pre-grasp and final grasp
      pickup_goal.desired_approach_distance = 0.1;
      pickup_goal.min_approach_distance = 0.05;
      //we will be lifting the object along the "vertical" direction
      //which is along the z axis in the base_link frame
      geometry_msgs::Vector3Stamped direction;
      direction.header.stamp = ros::Time::now();
      direction.header.frame_id = "base_link";
      direction.vector.x = 0;
      direction.vector.y = 0;
      direction.vector.z = 1;
      pickup_goal.lift.direction = direction;
      //request a vertical lift of 10cm after grasping the object
      pickup_goal.lift.desired_distance = 0.10;
      pickup_goal.lift.min_distance = 0.5;
      //do not use tactile-based grasping or tactile-based lift
      pickup_goal.use_reactive_lift = false;
      pickup_goal.use_reactive_execution = false;
  
      //send the goal
      pickup_client.sendGoal(pickup_goal);
      while (!pickup_client.waitForResult(ros::Duration(10.0)))
	{
	  ROS_INFO("Waiting for the pickup action...");
	}
      object_manipulation_msgs::PickupResult pickup_result = 
	*(pickup_client.getResult());
      if (pickup_client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
	{
	  ROS_ERROR("The pickup action has failed with result code %d", 
		    pickup_result.manipulation_result.value);
	  return false;
	}


      /*********************************************************************/
      /*********************************************************************/
      /******** Removed stuff about remembering the pick_up location ********/
      
        //remember where we picked the object up from
      geometry_msgs::PoseStamped pickup_location;
      //for unrecognized point clouds, the location of the object 
    //is considered to be the origin of the frame that the 
    //cluster is in
    pickup_location.header = object.cluster.header;
    pickup_location.header.frame_id = "base_link";
    //identity pose
    pickup_location.pose.orientation.w = 1;

  //create a place location, offset by 10 cm from the pickup location
  geometry_msgs::PoseStamped place_location = pickup_location;
  place_location.header.frame_id = "base_link";
  place_location.header.stamp = ros::Time::now();
  place_location.pose.position.x += 0.1;

  
      /********** Changed the place location coordinates **********/
  
      //create a place location at the bin location
     /*
      geometry_msgs::PoseStamped place_location;
      place_location.header.frame_id = "base_link";
      place_location.header.stamp = ros::Time::now();
      place_location.pose.position.x = binx;
      place_location.pose.position.y = biny;
      place_location.pose.position.z = binz + 0.06;
      */
      /********************************************************************/
      /*********************************************************************/

  
      //put the object down
      ROS_INFO("Calling the place action");
      object_manipulation_msgs::PlaceGoal place_goal;
      //place at the prepared location
      place_goal.place_locations.push_back(place_location);
      //the collision names of both the objects and the table
      //same as in the pickup action
      // place_goal.collision_object_name = 
      //   processing_call.response.collision_object_names.at(0); 
      place_goal.collision_support_surface_name = 
	processing_call.response.collision_support_surface_name;
      //information about which grasp was executed on the object, returned by
      //the pickup action
      place_goal.grasp = pickup_result.grasp;
      //use the right rm to place
      place_goal.arm_name = "right_arm";
      //padding used when determining if the requested place location
      //would bring the object in collision with the environment
      place_goal.place_padding = 0.0;
      //how much the gripper should retreat after placing the object
      place_goal.desired_retreat_distance = 0.1;
      place_goal.min_retreat_distance = 0.05;
      //we will be putting down the object along the "vertical" direction
      //which is along the z axis in the base_link frame
      direction.header.stamp = ros::Time::now();
      direction.header.frame_id = "base_link";
      direction.vector.x = 0;
      direction.vector.y = 0;
      direction.vector.z = -1;
      place_goal.approach.direction = direction;
      //request a vertical put down motion of 10cm before placing the object 
      place_goal.approach.desired_distance = 0.02;
      place_goal.approach.min_distance = 0.01;
      //we are not using tactile based placing
      place_goal.use_reactive_place = false;
      //send the goal
      place_client.sendGoal(place_goal);
      while (!place_client.waitForResult(ros::Duration(10.0)))
	{
	  ROS_INFO("Waiting for the place action...");
	}
      object_manipulation_msgs::PlaceResult place_result = 
	*(place_client.getResult());
      if (place_client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
	{
	  ROS_ERROR("Place failed with error code %d", 
		    place_result.manipulation_result.value);
	  return false;
	}

      //success!
      ROS_INFO("Success! Object moved.");
    }	
  }
  
  //  ros::spin();
    return 0;
}

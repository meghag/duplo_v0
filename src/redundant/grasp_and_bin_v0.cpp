#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <tabletop_object_detector/TabletopDetection.h>
#include <tabletop_collision_map_processing/TabletopCollisionMapProcessing.h>
#include <object_manipulation_msgs/PickupAction.h>
#include <object_manipulation_msgs/PlaceAction.h>

void pick_and_place(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  // ??? for (no. of clusters published) {
  	//  ??? Read in the cluster pcd and pick and place
  ROS_INFO("Calling the pickup action");
  object_manipulation_msgs::PickupGoal pickup_goal;
  //pass one of the graspable objects returned by the collision map processor
  pickup_goal.target = *msg;
  
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
  pickup_goal.lift.desired_distance = 0.1;
  pickup_goal.lift.min_distance = 0.05;
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
    return -1;
  }

  /********** Place in correct bin *********/

  //Hard-coding the bin locations for now
  float min_diff = abs(bin_color[0] - msg.points[0].rgb);
  int matching_bin_idx = 0;
  for (i = 1; i < n_bins; i++)
  {
	if (min_diff > abs(bin_color[i] - msg.points[0].rgb)) {
	  matching_bin_idx = i;
	  min_diff = abs(bin_color[0] - msg.points[0].rgb);
	}
  }

  //Obtain current point cloud from Kinect
   /* to be filled */

  //Segment out cylinders 
  //cylinder_segmentation(current_pcd);

  //put the object down
  ROS_INFO("Calling the place action");
  object_manipulation_msgs::PlaceGoal place_goal;
  //place at the prepared location
  place_goal.place_pose = bin_location[matching_bin_idx];
  
  //information about which grasp was executed on the object, returned by
  //the pickup action
  place_goal.grasp = pickup_result.grasp;
  //use the right rm to place
  place_goal.arm_name = "right_arm";
  //padding used when determining if the requested place location
  //would bring the object in collision with the environment
  place_goal.place_padding = 0.02;
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
  place_goal.approach.desired_distance = 0.1;
  place_goal.approach.min_distance = 0.05;
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
    return -1;
  }
  // ???If pick and place fails for one cluster, ignore and move on to the enxt cluster
  // } ??? end of for loop for pick and place
  // ??? return 0;
}

int grasp_and_bin()
{
  //initialize the ROS node
  ros::init(argc, argv, "pick_and_place_app");
  ros::NodeHandle nh;

  //set service and action names
  /*
  const std::string OBJECT_DETECTION_SERVICE_NAME = "/object_detection";
  const std::string COLLISION_PROCESSING_SERVICE_NAME = 
    "/tabletop_collision_map_processing/tabletop_collision_map_processing";
  */
   const std::string PICKUP_ACTION_NAME = 
    "/object_manipulator/object_manipulator_pickup";
  const std::string PLACE_ACTION_NAME = 
    "/object_manipulator/object_manipulator_place";

  //create service and action clients
/*
  ros::ServiceClient object_detection_srv;
  ros::ServiceClient collision_processing_srv;
*/
   actionlib::SimpleActionClient<object_manipulation_msgs::PickupAction> 
    pickup_client(PICKUP_ACTION_NAME, true);
  actionlib::SimpleActionClient<object_manipulation_msgs::PlaceAction> 
    place_client(PLACE_ACTION_NAME, true);

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

  //call object pickup

  /* ??????? Do it iteratively for each cluster. ??????????? /*
  /* ?????? Publish no. of clusters instead of each cluster point cloud ???????*/
  
  ros::Subscriber sub = nh.subscribe("grasp_duplo",10,pick_and_place);
  //object_manipulation_msgs/GraspableObject graspable;

  //success!
  ROS_INFO("Success! Object moved.");

	// ???? Call for latest point cloud and start from beginning ???? 
  return 0;
}
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

#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>
#include <move_arm_msgs/MoveArmAction.h>
#include <move_arm_msgs/utils.h>
#include <pr2_controllers_msgs/PointHeadAction.h>

typedef actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction> GripperClient;
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction> PointHeadClient;

class RobotHead
{
private:
  PointHeadClient* point_head_client_;

public:
  //! Action client initialization 
  RobotHead()
  {
    //Initialize the client for the Action interface to the head controller
    point_head_client_ = new PointHeadClient("/head_traj_controller/point_head_action", true);

    //wait for head controller action server to come up 
    while(!point_head_client_->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the point_head_action server to come up");
    }
  }

  ~RobotHead()
  {
    delete point_head_client_;
  }

  //! Points the high-def camera frame at a point in a given frame  
  void lookAt(std::string frame_id, double x, double y, double z)
  {
    //the goal message we will be sending
    pr2_controllers_msgs::PointHeadGoal goal;

    //the target point, expressed in the requested frame
    geometry_msgs::PointStamped point;
    point.header.frame_id = frame_id;
    point.point.x = x; point.point.y = y; point.point.z = z;
    goal.target = point;

    //we are pointing the high-def camera frame 
    //(pointing_axis defaults to X-axis)
    goal.pointing_frame = "high_def_frame";

    //take at least 0.5 seconds to get there
    goal.min_duration = ros::Duration(0.5);

    //and go no faster than 1 rad/s
    goal.max_velocity = 1.0;

    //send the goal
    point_head_client_->sendGoal(goal);

    //wait for it to get there (abort after 2 secs to prevent getting stuck)
    point_head_client_->waitForResult(ros::Duration(2));
  }

};

class Gripper{
private:
  GripperClient* gripper_client_;  

public:
  //Action client initialization
  Gripper(){

    //Initialize the client for the Action interface to the gripper controller
    //and tell the action client that we want to spin a thread by default
    gripper_client_ = new GripperClient("r_gripper_controller/gripper_action", true);
    
    //wait for the gripper action server to come up 
    while(!gripper_client_->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the r_gripper_controller/gripper_action action server to come up");
    }
  }

  ~Gripper(){
    delete gripper_client_;
  }

  //Open the gripper
  bool open(){
    pr2_controllers_msgs::Pr2GripperCommandGoal open;
    open.command.position = 0.08;
    open.command.max_effort = -1.0;  // Do not limit effort (negative)
    
    ROS_INFO("Sending open goal");
    gripper_client_->sendGoal(open);
    gripper_client_->waitForResult();
    if(gripper_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_INFO("The gripper opened!");
	  return true;
	}
    else {
      ROS_INFO("The gripper failed to open.");
	  return false;
	}
  }

};

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
  Gripper gripper;
  

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
      detection_call.request.return_models = false;
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
	  ROS_DEBUG("The tabletop detector detected the table, but found no objects");
	  //return false;
	}
	  detection_call.response.detection.clusters.clear();

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
      processing_call.request.take_static_collision_map = false;
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
	  ROS_DEBUG("Collision map processing returned no graspable objects");
	  //return false;
	}

      //call object pickup
      ROS_INFO("Calling the pickup action");
      object_manipulation_msgs::PickupGoal pickup_goal;
  
      //pass one of the graspable objects returned by the collission map processor

      /*********************************************************************/
      /*********************************************************************/
      /********** Changed *********/
      //pickup_goal.target = processing_call.response.graspable_objects.at(0);

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
      pickup_goal.lift.desired_distance = 0.15;
      pickup_goal.lift.min_distance = 0.1;
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
	  } else {
		  ROS_INFO("The pickup action succeeded.");
	  }


      /*********************************************************************/
      /*********************************************************************/
      /******** Removed stuff about remembering the pick_up location ********/
      
        //remember where we picked the object up from
      geometry_msgs::PoseStamped pickup_location;
      //for unrecognized point clouds, the location of the object is considered 
	  //to be the origin of the frame that the cluster is in
	  pickup_location.pose = pickup_result.grasp.grasp_pose;
	  pickup_location.header = object.cluster.header;
	  pickup_location.header.frame_id = "base_link";
	  //identity pose
//	  pickup_location.pose.orientation.w = 1;

	  //create a place location, offset by 10 cm from the pickup location
	  geometry_msgs::PoseStamped place_location = pickup_location;
	  place_location.header.frame_id = "base_link";
	  place_location.header.stamp = ros::Time::now();
	  place_location.pose.position.z -= 0.02;
  
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
		ROS_INFO("Time stamp and frame_id of place_location: %d %s", 
		          place_location.header.stamp.sec, place_location.header.frame_id.c_str());
		ROS_INFO("place_location: %f %f %f", 
		          place_location.pose.position.x, place_location.pose.position.y, 
		         place_location.pose.position.z);
		
      object_manipulation_msgs::PlaceGoal place_goal;
      //place at the prepared location
      place_goal.place_locations.push_back(place_location);
      //the collision names of the table same as in the pickup action
      // place_goal.collision_object_name = 
      //   processing_call.response.collision_object_names.at(0); 
      place_goal.collision_support_surface_name = 
		processing_call.response.collision_support_surface_name;
	  place_goal.allow_gripper_support_collision = true;
      //information about which grasp was executed on the object, returned by
      //the pickup action
      place_goal.grasp = pickup_result.grasp;
      //use the right arm to place
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
      place_goal.approach.desired_distance = 0.1;
      place_goal.approach.min_distance = 0.05;
      //we are not using tactile based placing
      place_goal.use_reactive_place = false;
      //send the goal
      place_client.sendGoal(place_goal);
      while (!place_client.waitForResult(ros::Duration(10.0))) {
		ROS_INFO("Waiting for the place action...");
	  }
      object_manipulation_msgs::PlaceResult place_result = 
		*(place_client.getResult());
      if (place_client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
	  {
		ROS_ERROR("Place failed with error code %d", place_result.manipulation_result.value);
		//return false;
		  //if (gripper.open()) {
			  actionlib::SimpleActionClient<move_arm_msgs::MoveArmAction> move_arm("move_right_arm",true);
			  move_arm.waitForServer();
			  ROS_INFO("Connected to server");
			  move_arm_msgs::MoveArmGoal goalA;

			  goalA.motion_plan_request.group_name = "right_arm";
			  goalA.motion_plan_request.num_planning_attempts = 1;
			  goalA.motion_plan_request.planner_id = std::string("");
			  goalA.planner_service_name = std::string("ompl_planning/plan_kinematic_path");
			  goalA.motion_plan_request.allowed_planning_time = ros::Duration(5.0);

			  motion_planning_msgs::SimplePoseConstraint desired_pose;
			  desired_pose.header.frame_id = "torso_lift_link";
			  desired_pose.link_name = "r_wrist_roll_link";
			  desired_pose.pose.position.x =  0.6; //0.75;
			  desired_pose.pose.position.y = -0.5;//-0.188;
			  desired_pose.pose.position.z = 0;

			  desired_pose.pose.orientation.x = 0.0;
			  desired_pose.pose.orientation.y = 0.0;
			  desired_pose.pose.orientation.z = 0.0;
			  desired_pose.pose.orientation.w = 1.0;

			  desired_pose.absolute_position_tolerance.x = 0.02;
			  desired_pose.absolute_position_tolerance.y = 0.02;
			  desired_pose.absolute_position_tolerance.z = 0.02;

			  desired_pose.absolute_roll_tolerance = 0.04;
			  desired_pose.absolute_pitch_tolerance = 0.04;
			  desired_pose.absolute_yaw_tolerance = 0.04;

			  move_arm_msgs::addGoalConstraintToMoveArmGoal(desired_pose,goalA);

			  if (nh.ok())
			  {
				  bool finished_within_time = false;
				  move_arm.sendGoal(goalA);
				  finished_within_time = move_arm.waitForResult(ros::Duration(200.0));
				  if (!finished_within_time)
				  {
					  move_arm.cancelGoal();
					  ROS_INFO("Timed out achieving goal A");
				  }
				  else
				  {
					  actionlib::SimpleClientGoalState state = move_arm.getState();
					  bool success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
					  if(success)
						  ROS_INFO("Action finished: %s",state.toString().c_str());
					  else
						  ROS_INFO("Action failed: %s",state.toString().c_str());
				  }
			  }
			  gripper.open();
			  RobotHead head;
  			  head.lookAt("base_link", 0.2, 0.0, 1.0);
		  //}
		  
	  } else {
		  //success!
		  ROS_INFO("Success! Object moved.");
	  }
    }	
  }
  
    ros::spin();
    return 0;
}

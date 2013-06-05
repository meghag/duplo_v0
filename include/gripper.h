#ifndef GRIPPER_H
#define GRIPPER_H

#include <ros/ros.h>
#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>
#include <actionlib/client/simple_action_client.h>

// Our Action interface type, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction> GripperClient;

class Gripper{
	private:
		GripperClient* gripper_client_;  

	public:
		//Action client initialization
		Gripper (void);
		~Gripper (void);

		//Open the gripper
		void open (void);
		void close (void);
};

#endif
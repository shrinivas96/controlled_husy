#include <ros/ros.h>
#include "smb_highlevel_controller/SmbHighlevelController.hpp"

int main(int argc, char **argv)
{

	// ROS always initializes here.
	// This is the main node that starts all the process
	ros::init(argc, argv, "smb_highlevel_controller");

	// private node handle to be passed to the package class
	ros::NodeHandle nodeHandle("~");

	// instantiate object of main class and pass the node handle
	// for my ref: namespace::className
	smb_highlevel_controller::SmbHighlevelController huskyObject(nodeHandle);

	// wait for incoming messages. returns only when node is shutdown.
	ros::spin();
	return 0;
}

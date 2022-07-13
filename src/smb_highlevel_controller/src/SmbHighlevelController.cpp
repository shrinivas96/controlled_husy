#include <smb_highlevel_controller/SmbHighlevelController.hpp>

namespace smb_highlevel_controller
{

	/*!
	 * Constructor.
	 */
	SmbHighlevelController::SmbHighlevelController(ros::NodeHandle &nodeHandle) : nodeHandle_(nodeHandle)
	{
		currentRange_ = 0;
		obstacleMid_ = 0;

		// check if parameters can be read
		if (!readParameters())
		{
			ROS_ERROR("The parameteres could not be loaded. Shutting down.");
			ros::requestShutdown();
		}

		laserScanSub_ = nodeHandle_.subscribe(laserScanTopic_, queue_size_, &SmbHighlevelController::findPillar, this);
		cmdVelPub_ = nodeHandle_.advertise<geometry_msgs::Twist>(cmdVelTopic_, 10);

		ros::Rate loopRate(10);
		geometry_msgs::Twist newCmdVel;

		float desiredOrientation = 0;
		float angError, kp_z = -0.8;

		float desiredRange = 5.0;
		float distError = 0, kp_x = 4;

		while (ros::ok())
		{
			angError = desiredOrientation - obstacleMid_;
			distError = desiredRange - currentRange_;

			newCmdVel.angular.z = kp_z * angError;
			newCmdVel.linear.x = kp_x * distError;

			cmdVelPub_.publish(newCmdVel);

			ros::spinOnce();
			loopRate.sleep();
		}
	}

	/*!
	 * Destructor.
	 */
	SmbHighlevelController::~SmbHighlevelController()
	{
	}

	/*
	 * read parameters and see if they are present
	 */
	bool SmbHighlevelController::readParameters()
	{
		/*
		Function could to be optimised. Intentionally made it a bit long to understand what is going on
		*/

		// getParam returns true and sets the value if parameter found.
		// the location of all parameters is specified in the launch file.
		bool subScan = nodeHandle_.getParam("subscriber_topic", laserScanTopic_);
		bool subQueue = nodeHandle_.getParam("queue_size", queue_size_);
		bool pubCmdVel = nodeHandle_.getParam("vel_pub", cmdVelTopic_);

		// check if both the parameters are available, then proceed
		if (subQueue && subScan && pubCmdVel)
		{
			ROS_INFO_STREAM("All parameters were found.");
			return true;
		}
		else
		{
			ROS_INFO_STREAM("Could not find all paramters.");
			return false;
		}
	}

	/*
	 * callback function to print number of points in 3d point cloid when message received on /rslidar_points
	 */
	void SmbHighlevelController::findPillar(const sensor_msgs::LaserScan &laserScan)
	{
		float startObstacle = 0, endObstacle = 0;
		int flag = 0, obstMidIndx = 0;

		int numReadings = laserScan.ranges.size();

		float range = 0, orient = laserScan.angle_min;

		std::vector<float> obstacleRanges, obstacleIndices;

		// looping through all ranges in this scan
		for (int i = 0; i < numReadings; i++)
		{
			range = laserScan.ranges[i];
			if (!isinf(range))
			{
				// finding the first non-inf range; pillar starts here
				if (flag == 0)
				{
					flag = 1;
					startObstacle = orient;
					endObstacle = orient;
				}
				obstacleIndices.emplace_back(i);
				obstacleRanges.emplace_back(range);
				endObstacle += laserScan.angle_increment;
			}
			orient += laserScan.angle_increment;
		} // looping through the scan

		if (flag == 0)
		{
			currentRange_ = 5.0;
			obstacleMid_ = 0.0;
		}

		else if (flag == 1)
		{
			obstacleMid_ = (startObstacle + endObstacle) / 2;
			obstMidIndx = (int)floor(obstacleIndices.size() / 2);
			currentRange_ = obstacleRanges[obstMidIndx];
		}
	}

} /* namespace */

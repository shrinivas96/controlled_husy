#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

namespace smb_highlevel_controller
{

	/*!
	 * Class containing the Husky Highlevel Controller
	 */
	class SmbHighlevelController
	{
	public:
		/*!
		 * Constructor.
		 */
		SmbHighlevelController(ros::NodeHandle &nodeHandle);

		/*!
		 * Destructor.
		 */
		virtual ~SmbHighlevelController();

	private:
		ros::NodeHandle nodeHandle_;   // node handle to access the ROS network
		ros::Subscriber laserScanSub_; // to subscribe to point clouds -> /rslidar_points
		ros::Publisher cmdVelPub_;

		// to store topic name from parameter server
		std::string laserScanTopic_;
		std::string cmdVelTopic_;
		int queue_size_;

		//some internal variables needed for finding the pillar
		float obstacleMid_;
		float currentRange_;

		bool readParameters(); // Check if parameters can be read
		void findPillar(const sensor_msgs::LaserScan &laserScan);
	};

} /* namespace */
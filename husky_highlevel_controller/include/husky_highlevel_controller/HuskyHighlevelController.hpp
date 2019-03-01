#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <std_srvs/SetBool.h>

namespace husky_highlevel_controller {

/*!
 * Class containing the Husky Highlevel Controller
 */
class HuskyHighlevelController {
public:
	/*!
	 * Constructor.
	 */
	HuskyHighlevelController(ros::NodeHandle& nodeHandle);

	/*!
	 * Destructor.
	 */
	virtual ~HuskyHighlevelController();

private:

	void LaserCallback(const sensor_msgs::LaserScan& scan);
	bool ServiceCallback(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response);

	ros::NodeHandle nodeHandle_;
	ros::Subscriber subscriber_;
	ros::Publisher publisher_;
	ros::Publisher visual_pub_;
	ros::ServiceServer service_;

	bool husky_stop_;

	float p_gain_;
};

} /* namespace */

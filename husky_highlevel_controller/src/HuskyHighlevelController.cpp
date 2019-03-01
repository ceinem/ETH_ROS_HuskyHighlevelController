#include "husky_highlevel_controller/HuskyHighlevelController.hpp"
#include "sensor_msgs/LaserScan.h"



namespace husky_highlevel_controller {

HuskyHighlevelController::HuskyHighlevelController(ros::NodeHandle& nodeHandle) :
  nodeHandle_(nodeHandle)
{

  husky_stop_ = false;

  std::string topic;
  if(!nodeHandle.getParam("husky_subscriber_topic", topic))
  {
    ROS_ERROR("Could not find topic  parameter!");
  }

  int queue_size;
  if(!nodeHandle.getParam("husky_subscriber_queue_size", queue_size))
  {
    ROS_ERROR("Could not find topic  parameter!");
  }

  //float p_gain;
  if(!nodeHandle.getParam("husky_p_gain", p_gain_))
  {
    ROS_ERROR("Could not find topic  parameter!");
  }

  subscriber_ = nodeHandle_.subscribe(topic, queue_size, &HuskyHighlevelController::LaserCallback, this);
  publisher_ = nodeHandle_.advertise<geometry_msgs::Twist>("/cmd_vel",1);
  visual_pub_ = nodeHandle_.advertise<visualization_msgs::Marker>( "/visualization_marker", 100 );

  service_ = nodeHandle_.advertiseService("/cmd_husky_stop", &HuskyHighlevelController::ServiceCallback, this);

}

HuskyHighlevelController::~HuskyHighlevelController()
{
}

void HuskyHighlevelController::LaserCallback(const sensor_msgs::LaserScan& scan)
{
  geometry_msgs::Twist control_vel;


  float angle_increment = scan.angle_increment;
  float angle_min = scan.angle_min;


  float min_range = scan.ranges[0];
  int min_range_index = 0;
  float min_range_angle;

  //if(scan.ranges.size() > 0 ) {
    for (int i = 0; i < (scan.ranges).size(); ++i) {
      if(scan.ranges[i] < min_range) {
        min_range = scan.ranges[i];
        min_range_index = i;
      }
    }
    min_range_angle = angle_min + angle_increment * min_range_index;
    //ROS_INFO_STREAM("Minimum is: " << min_range);
    //std::cout << "angle_error = " << min_range_angle << std::endl;

    float angle_error = -min_range_angle;

    if(min_range < 1000){
      if(husky_stop_) {
        control_vel.angular.z = 0.0;
        control_vel.linear.x = 0.0;
      } else {
        control_vel.angular.z = p_gain_ * angle_error;
        control_vel.linear.x = 10.0;
      }

      publisher_.publish(control_vel);


      visualization_msgs::Marker pillar_marker;
      pillar_marker.header.frame_id = "base_link";
      pillar_marker.header.stamp = ros::Time();
      pillar_marker.ns = "marker";
      pillar_marker.id = 1;
      pillar_marker.type = visualization_msgs::Marker::SPHERE;
      pillar_marker.action = visualization_msgs::Marker::ADD;
      pillar_marker.pose.position.x = min_range * std::cos(min_range_angle) + 0.337;
      pillar_marker.pose.position.y = -min_range * std::sin(min_range_angle);
      pillar_marker.pose.position.z = 0.308;
      pillar_marker.pose.orientation.x = 0.0;
      pillar_marker.pose.orientation.y = 0.0;
      pillar_marker.pose.orientation.z = 0.0;
      pillar_marker.pose.orientation.w = 1.0;
      pillar_marker.scale.x = 0.1;
      pillar_marker.scale.y = 0.1;
      pillar_marker.scale.z = 1;
      pillar_marker.color.a = 1.0;
      pillar_marker.color.r = 1.0;
      pillar_marker.color.g = 0.0;
      pillar_marker.color.b = 0.0;
      visual_pub_.publish( pillar_marker );
    }

}


bool HuskyHighlevelController::ServiceCallback(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response){

  if(request.data) {
    husky_stop_ = true;
    response.message = "Husky stopped";
  } else {
    husky_stop_ = false;
    response.message = "Husky continues";
  }

  response.success = true;
  return true;
}



} /* namespace */


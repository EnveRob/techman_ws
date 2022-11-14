#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include <iostream>
#include <cstring>
#include <string>
using namespace std;
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void feedbackCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
  // ROS_INFO("I heard x: [%s]", to_string(msg->x));
  // ROS_INFO("I heard y: [%s]", to_string(msg->y));
  // ROS_INFO("I heard z: [%s]", to_string(msg->z));
  printf ("x: %g\n", msg->x); 
  printf ("y: %g\n", msg->y); 
  printf ("z: %g\n", msg->z); 

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Enverob_feedback_controller");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("force_data", 100, feedbackCallback);

  ros::spin();
  return 0;
}
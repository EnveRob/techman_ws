#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"

void forceCallback(const geometry_msgs::Vector3& msg)
{
  ROS_INFO("Force_x: %f", msg.x);
  ROS_INFO("Force_y: %f", msg.y);
  ROS_INFO("Force_z: %f", msg.z);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "arduino_listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("force_data", 1000, forceCallback);
  ros::spin();

  return 0;
}

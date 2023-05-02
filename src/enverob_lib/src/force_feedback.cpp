#include "force_feedback.h"

namespace force_feedback
{
  void ForceCallback::forceCallback(const geometry_msgs::Vector3 &msg)
  {
    ROS_INFO("Force (x, y, z): (%2f, %2f, %2f)",
             msg.x, msg.y, msg.z);

    force_value_.x = msg.x;
    force_value_.y = msg.y;
    force_value_.z = msg.z;
  }

  geometry_msgs::Vector3 ForceCallback::getForceValue() const
  {
    return force_value_;
  }
}
#include "force_feedback.h"
#include "arm_move.h"
#include "new_frame.h"

namespace force_feedback
{
  void ForceCallback::shutCallback(const std_msgs::Bool::ConstPtr &msg)
  {
    if (msg->data == 1)
    {
      ROS_ERROR("Gripper shut down");
      ROS_INFO("Move to initial position");
      tf::StampedTransform targetTransform;
      new_frame::waitforTransform("mailbox_opening_offset", targetTransform);
      arm_move::setTargetPosition(nh_, *move_group_ptr_, plan_, targetTransform);
      std::vector<double> target_joint = {-M_PI / 180 * 135, -M_PI / 180 * 30, M_PI / 180 * 150, -M_PI / 180 * 120, M_PI / 2, -M_PI / 2};
      arm_move::setJointangle(nh_, *move_group_ptr_, plan_, target_joint);
      ros::shutdown();
    }
  }

  void ForceCallback::forceCallback(const geometry_msgs::Vector3 &msg)
  {
    // printf("Force (x, y, z): (%2f, %2f, %2f)\n",
    //        msg.x, msg.y, msg.z);

    force_value_.x = msg.x;
    force_value_.y = msg.y;
    force_value_.z = msg.z;
  }

  geometry_msgs::Vector3 ForceCallback::getForceValue() const
  {
    return force_value_;
  }
}
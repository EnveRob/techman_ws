#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

// const double gripper_offset = 0.10;
namespace new_frame
{
  void fixedFrame_add(std::vector<double> frame_pose,
                      std::string reference_frame,
                      std::string target_frame)
  {
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped ref2target;

    // ------------------ 設定相對參考至目標的轉換 ------------------
    ref2target.transform.translation.x = frame_pose[0];
    ref2target.transform.translation.y = frame_pose[1];
    ref2target.transform.translation.z = frame_pose[2];
    ref2target.transform.rotation.x = frame_pose[3];
    ref2target.transform.rotation.y = frame_pose[4];
    ref2target.transform.rotation.z = frame_pose[5];
    ref2target.transform.rotation.w = frame_pose[6];

    // 设置 transform 的时间戳、参考系名称和子参考系名称
    ref2target.header.stamp = ros::Time::now();
    ref2target.header.frame_id = reference_frame;
    ref2target.child_frame_id = target_frame;

    // 发布 transform
    static_broadcaster.sendTransform(ref2target);

    // 聽取轉換
    tf::TransformListener listener;
    tf::StampedTransform target_fixed;

    while (ros::ok())
    {
      try
      {
        listener.waitForTransform("base", target_frame, ros::Time(0), ros::Duration(3.0));
        listener.lookupTransform("base", target_frame, ros::Time(0), target_fixed);
        ROS_INFO("\nMailbox before rotated \n(x, y, z): %.2f, %.2f, %.2f, \n(qx, qy, qz, qw): %.2f, %.2f, %.2f, %.2f",
                 target_fixed.getOrigin().getX(),
                 target_fixed.getOrigin().getY(),
                 target_fixed.getOrigin().getZ(),
                 target_fixed.getRotation().getX(),
                 target_fixed.getRotation().getY(),
                 target_fixed.getRotation().getZ(),
                 target_fixed.getRotation().getW());
        break;
      }
      catch (tf::TransformException &ex)
      {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
      }
    }

    geometry_msgs::TransformStamped base2target;
    base2target.transform.rotation.x = target_fixed.getRotation().getX();
    base2target.transform.rotation.y = target_fixed.getRotation().getY();
    base2target.transform.rotation.z = target_fixed.getRotation().getZ();
    base2target.transform.rotation.w = target_fixed.getRotation().getW();
    base2target.transform.translation.x = target_fixed.getOrigin().getX();
    base2target.transform.translation.y = target_fixed.getOrigin().getY();
    base2target.transform.translation.z = target_fixed.getOrigin().getZ();

    // 设置 transform 的时间戳、参考系名称和子参考系名称
    base2target.header.stamp = ros::Time::now();
    base2target.header.frame_id = "base";
    base2target.child_frame_id = target_frame;

    // 发布 transform
    static_broadcaster.sendTransform(base2target);
  }
}
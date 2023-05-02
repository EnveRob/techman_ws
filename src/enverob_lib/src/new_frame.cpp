#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

namespace new_frame
{
  void waitforTransform(std::string target_frame, tf::StampedTransform target_transform)
  {
    tf::TransformListener listener;
    while (ros::ok())
    {
      try
      {
        listener.waitForTransform("base", target_frame, ros::Time(0), ros::Duration(3.0));
        listener.lookupTransform("base", target_frame, ros::Time(0), target_transform);
        ROS_INFO("Target positon related to base \n(x, y, z): %.2f, %.2f, %.2f, \n(qx, qy, qz, qw): %.2f, %.2f, %.2f, %.2f",
                 target_transform.getOrigin().getX(),
                 target_transform.getOrigin().getY(),
                 target_transform.getOrigin().getZ(),
                 target_transform.getRotation().getX(),
                 target_transform.getRotation().getY(),
                 target_transform.getRotation().getZ(),
                 target_transform.getRotation().getW());
        break;
      }
      catch (tf::TransformException &ex)
      {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
      }
    }
  }

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

    ROS_INFO("\nMailbox positon related to reference \n(x, y, z): %.2f, %.2f, %.2f, \n(qx, qy, qz, qw): %.2f, %.2f, %.2f, %.2f",
             frame_pose[0],
             frame_pose[1],
             frame_pose[2],
             frame_pose[3],
             frame_pose[4],
             frame_pose[5],
             frame_pose[6]);

    // 设置 transform 的时间戳、参考系名称和子参考系名称
    ref2target.header.stamp = ros::Time::now();
    ref2target.header.frame_id = reference_frame;
    ref2target.child_frame_id = target_frame;

    // 发布 transform
    static_broadcaster.sendTransform(ref2target);

    // 聽取轉換
    tf::StampedTransform target_fixed;
    waitforTransform(target_frame, target_fixed);

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
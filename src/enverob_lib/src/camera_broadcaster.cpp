#include <ros/ros.h>
#include "std_msgs/String.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

const double gripper_offset = 0.10;

void cameraCallback(const std_msgs::String::ConstPtr &msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());

  std::stringstream ss(msg->data.c_str());
  std::string token;
  std::vector<std::string> tokens;
  while (std::getline(ss, token, ' '))
  {
    tokens.push_back(token);
  }
  std::vector<double> v;
  for (const auto &token : tokens)
  {
    v.push_back(std::stod(token));
  }

  ROS_INFO("I heard: [%f]", v[0]);
  ROS_INFO("I heard: [%f]", v[1]);
  ROS_INFO("I heard: [%f]", v[2]);
  ROS_INFO("I heard: [%f]", v[3]);
  ROS_INFO("I heard: [%f]", v[4]);
  ROS_INFO("I heard: [%f]", v[5]);

  static tf2_ros::StaticTransformBroadcaster static_broadcaster;
  // static_broadcaster.sendTransform(std::vector<geometry_msgs::TransformStamped>());
  // static tf2_ros::StaticTransformBroadcaster static_br_base2mailbox;
  // static_br_base2mailbox.sendTransform(std::vector<geometry_msgs::TransformStamped>());

  geometry_msgs::TransformStamped mail2camera;

  tf::Quaternion q;
  q.setRPY(v[3], v[4], v[5]); // 以弧度為單位
  mail2camera.transform.rotation.x = q.x();
  mail2camera.transform.rotation.y = q.y();
  mail2camera.transform.rotation.z = q.z();
  mail2camera.transform.rotation.w = q.w();
  mail2camera.transform.translation.x = v[0];
  mail2camera.transform.translation.y = v[1];
  mail2camera.transform.translation.z = v[2] - gripper_offset;

  // 將座標改為z軸向上
  q.setX(mail2camera.transform.rotation.x);
  q.setY(mail2camera.transform.rotation.y);
  q.setZ(mail2camera.transform.rotation.z);
  q.setW(mail2camera.transform.rotation.w);

  // 定義旋轉軸和旋轉角度
  tf::Vector3 axis(1, 0, 0);
  double angle = -M_PI / 2;

  // 將旋轉軸和旋轉角度轉換為四元數
  tf::Quaternion rotation;
  rotation.setRotation(axis, angle);

  q *= rotation;
  q.normalize();

  mail2camera.transform.rotation.x = q.x();
  mail2camera.transform.rotation.y = q.y();
  mail2camera.transform.rotation.z = q.z();
  mail2camera.transform.rotation.w = q.w();

  // 设置 transform 的时间戳、参考系名称和子参考系名称
  mail2camera.header.stamp = ros::Time::now();
  mail2camera.header.frame_id = "camera";
  mail2camera.child_frame_id = "mailbox";

  // 发布 transform
  static_broadcaster.sendTransform(mail2camera);

  // 聽取轉換
  tf::TransformListener listener;
  tf::StampedTransform mailbox_frame;

  while (ros::ok())
  {
    try
    {
      listener.waitForTransform("base", "mailbox", ros::Time(0), ros::Duration(3.0));
      listener.lookupTransform("base", "mailbox", ros::Time(0), mailbox_frame);
      ROS_INFO("\nMailbox before rotated \n(x, y, z): %.2f, %.2f, %.2f, \n(qx, qy, qz, qw): %.2f, %.2f, %.2f, %.2f",
               mailbox_frame.getOrigin().getX(),
               mailbox_frame.getOrigin().getY(),
               mailbox_frame.getOrigin().getZ(),
               mailbox_frame.getRotation().getX(),
               mailbox_frame.getRotation().getY(),
               mailbox_frame.getRotation().getZ(),
               mailbox_frame.getRotation().getW());
      break;
    }
    catch (tf::TransformException &ex)
    {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
    }
  }

  geometry_msgs::TransformStamped base2mailbox;
  base2mailbox.transform.rotation.x = mailbox_frame.getRotation().getX();
  base2mailbox.transform.rotation.y = mailbox_frame.getRotation().getY();
  base2mailbox.transform.rotation.z = mailbox_frame.getRotation().getZ();
  base2mailbox.transform.rotation.w = mailbox_frame.getRotation().getW();
  base2mailbox.transform.translation.x = mailbox_frame.getOrigin().getX();
  base2mailbox.transform.translation.y = mailbox_frame.getOrigin().getY();
  base2mailbox.transform.translation.z = mailbox_frame.getOrigin().getZ();

  // static_broadcaster.sendTransform(std::vector<geometry_msgs::TransformStamped>());

  // 设置 transform 的时间戳、参考系名称和子参考系名称
  base2mailbox.header.stamp = ros::Time::now();
  base2mailbox.header.frame_id = "base";
  base2mailbox.child_frame_id = "mailbox";

  // 发布 transform
  // static_br_base2mailbox.sendTransform(base2mailbox);
  static_broadcaster.sendTransform(base2mailbox);

  // 聽取轉換
  tf::TransformListener listener2;
  tf::StampedTransform mailbox_fixed;
  while (ros::ok())
  {
    try
    {
      listener2.waitForTransform("base", "mailbox", ros::Time(0), ros::Duration(3.0));
      listener2.lookupTransform("base", "mailbox", ros::Time(0), mailbox_fixed);
      ROS_INFO("\nMailbox after rotated \n(x, y, z): %.2f, %.2f, %.2f, \n(qx, qy, qz, qw): %.2f, %.2f, %.2f, %.2f",
               mailbox_fixed.getOrigin().getX(),
               mailbox_fixed.getOrigin().getY(),
               mailbox_fixed.getOrigin().getZ(),
               mailbox_fixed.getRotation().getX(),
               mailbox_fixed.getRotation().getY(),
               mailbox_fixed.getRotation().getZ(),
               mailbox_fixed.getRotation().getW());
      break;
    }
    catch (tf::TransformException &ex)
    {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "camera_broadcaster");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("camera_data", 10, &cameraCallback);
  ROS_INFO("subsrciber running");
  ros::spin();
  return 0;
}
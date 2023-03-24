#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "std_msgs/String.h"
#include <string>
#include <sstream>
#include <vector>
#include <iostream>



void cameraCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("I heard: [%s]", msg->data.c_str());
    static tf::TransformBroadcaster broadcaster; //创建tf广播器
    tf::Transform base2laser;
    std::stringstream ss(msg->data.c_str());
    std::string token;
    std::vector<std::string> tokens;
    while (std::getline(ss, token, ' ')) {
        tokens.push_back(token);
    }
    std::vector<double> v;
    for (const auto& token : tokens) {
        v.push_back(std::stod(token));
    }
    tf::Quaternion q;
    ROS_INFO("I heard: [%f]", v[0]);
    ROS_INFO("I heard: [%f]", v[1]);
    ROS_INFO("I heard: [%f]", v[2]);
    ROS_INFO("I heard: [%f]", v[3]);
    ROS_INFO("I heard: [%f]", v[4]);
    ROS_INFO("I heard: [%f]", v[5]);

    q.setRPY(v[3],v[4],v[5]);
    // q.setRPY(0.05,0.05,0.01);
    base2laser.setRotation(q);              //设置旋转坐标
    base2laser.setOrigin(tf::Vector3(v[0],v[1],v[2]));//设平移坐标，mailbox在camera的(1,0,0)位置
    // base2laser.setOrigin(tf::Vector3(0,1,1));//设平移坐标，mailbox在camera的(1,0,0)位置
    
    //循环发布坐标变换，两种方式
    broadcaster.sendTransform(tf::StampedTransform(base2laser,ros::Time::now(),"camera","mailbox"));
    //broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0s, 0, 0, 0), tf::Vector3(1, 0.0, 0)),ros::Time::now(),"base_link", "base_laser"));
    // loop_rate.sleep();
    
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "camera_broadcaster");//初始化ROS节点与节点名称
    ros::NodeHandle n;                    //创建节点的句柄 
    // ros::Rate loop_rate(10);
    
    ros::Subscriber sub = n.subscribe("camera_data", 10, &cameraCallback);
    //控制节点运行的频率,与loop.sleep共同使用

    ros::spin();  
    return 0;
}
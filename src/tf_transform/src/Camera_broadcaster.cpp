#include <ros/ros.h>
#include "std_msgs/String.h"
#include <tf/transform_broadcaster.h>

#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

const double gripper_offset = 0.135;


void cameraCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("I heard: [%s]", msg->data.c_str());
    
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

    ROS_INFO("I heard: [%f]", v[0]);
    ROS_INFO("I heard: [%f]", v[1]);
    ROS_INFO("I heard: [%f]", v[2]);
    ROS_INFO("I heard: [%f]", v[3]);
    ROS_INFO("I heard: [%f]", v[4]);
    ROS_INFO("I heard: [%f]", v[5]);

    // --------------------------------------------------------------------------------
    // static tf::TransformBroadcaster broadcaster; //创建tf广播器
    // tf::Transform mail2camera;
    // tf::Quaternion q;
    // q.setRPY(v[3],v[4],v[5]);                   //以弧度為單位
    // mail2camera.setRotation(q);                 //设置旋转坐标
    // mail2camera.setOrigin(tf::Vector3(v[0],v[1],v[2]-gripper_offset));//设平移坐标，mailbox在camera的(1,0,0)位置
    //mail2camera.setOrigin(tf::Vector3(0,1,1));//设平移坐标，mailbox在camera的(1,0,0)位置
    // ros::Time start_time = ros::Time::now();
    // //循环发布坐标变换，两种方式
    // while(ros::Time::now()-start_time <= ros::Duration(5))
    // {
    //     broadcaster.sendTransform(tf::StampedTransform(mail2camera,ros::Time::now(),"camera","mailbox"));
    //     //broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0, 0, 0, 0), tf::Vector3(1, 0.0, 0)),ros::Time::now(),"base_link", "base_laser"));
    //     ROS_INFO("finish broadcast");
    //     ros::Duration(0.1).sleep();
    // }

    // --------------------------------------------------------------------------------
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped mail2camera;

    // 创建一个 tf::Transform 对象
    // tf::Transform mail2camera;
    tf::Quaternion q;
    q.setRPY(v[3],v[4],v[5]);                   //以弧度為單位
    mail2camera.transform.rotation.x = q.x();
    mail2camera.transform.rotation.y = q.y();
    mail2camera.transform.rotation.z = q.z();
    mail2camera.transform.rotation.w = q.w();
    mail2camera.transform.translation.x = v[0];
    mail2camera.transform.translation.y = v[1];
    mail2camera.transform.translation.z = v[2]-gripper_offset;

    // 设置 transform 的时间戳、参考系名称和子参考系名称
    mail2camera.header.stamp = ros::Time::now();
    mail2camera.header.frame_id = "camera";
    mail2camera.child_frame_id = "mailbox";

    // 发布 transform
    static_broadcaster.sendTransform(mail2camera);

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "camera_broadcaster"); //初始化ROS节点与节点名称
    ros::NodeHandle n;
    // ros::Rate loop_rate(10);
    
    ros::Subscriber sub = n.subscribe("camera_data", 10, &cameraCallback);
    //控制节点运行的频率,与loop.sleep共同使用
    ROS_INFO("subsrcober running");
    ros::spin();  
    return 0;
}
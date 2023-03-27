#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/String.h>


// TM Driver header
#include "tm_msgs/SetPositions.h"

const double gripper_offset = 0.135;
const double searching_rate = 10;


int main(int argc,char** argv)
{
    ros::init(argc,argv,"arm_listener");       //初始化ROS节点与节点名称
    ros::NodeHandle n;                         //创建节点的句柄
    ros::Rate loop_rate(1);                   //控制节点运行的频率,与loop.sleep共同使用

    //创建一个监听器，监听所有tf变换，缓冲10s
    tf::TransformListener listener;
    ros::ServiceClient client = n.serviceClient<tm_msgs::SetPositions>("tm_driver/set_positions");
    tm_msgs::SetPositions srv;
    tf::StampedTransform laserTransform;
    double searching_direction = 0;    
    //Request

    srv.request.motion_type = tm_msgs::SetPositions::Request::PTP_J;
    srv.request.positions.push_back(0);
    srv.request.positions.push_back(0);
    srv.request.positions.push_back(0);
    srv.request.positions.push_back(0);
    srv.request.positions.push_back(0);
    srv.request.positions.push_back(0);
    srv.request.velocity = 2;//rad/s
    srv.request.acc_time = 0.5;
    srv.request.blend_percentage = 10;
    srv.request.fine_goal  = false;
    std::cout << "current direction" << searching_direction << std::endl; 

    if (client.call(srv))                             
    {
        if (srv.response.ok) ROS_INFO_STREAM("SetPositions to robot");
        else ROS_WARN_STREAM("SetPositions to robot , but response not yet ok ");
    }
    else
    {
        ROS_ERROR_STREAM("Error SetPositions to robot");
        return 1;
    } 

    for (double i = 0;i <= 3.14; i+=0.0314)
    {
        srv.request.motion_type = tm_msgs::SetPositions::Request::PTP_J;
        srv.request.positions[0] = 0;
        srv.request.positions[1] = 0;
        srv.request.positions[2] = 0;
        srv.request.positions[3] = 0;
        srv.request.positions[4] = i;
        srv.request.positions[5] = 0;
        srv.request.velocity = 2;//rad/s
        srv.request.acc_time = 0.5;
        srv.request.blend_percentage = 10;
        srv.request.fine_goal  = false;
        std::cout << "current direction" << i << std::endl;  
        
        // searching_direction = searching_direction + searching_rate;

        if (client.call(srv))                             
        {
            if (srv.response.ok) ROS_INFO_STREAM("SetPositions to robot");
            else ROS_WARN_STREAM("SetPositions to robot , but response not yet ok ");
        }
        else
        {
            ROS_ERROR_STREAM("Error SetPositions to robot");
            return 1;
        }

        if (listener.waitForTransform("base","mailbox",ros::Time(0),ros::Duration(1))) //等待10s，如果10s之后都还没收到消息，那么之前的消息就被丢弃掉。
        {
            
            listener.lookupTransform("base","mailbox", ros::Time(0), laserTransform);
            std::cout << "laserTransform.getOrigin().getX(): " << laserTransform.getOrigin().getX() << std::endl;
            std::cout << "laserTransform.getOrigin().getY(): " << laserTransform.getOrigin().getY() << std::endl;
            std::cout << "laserTransform.getOrigin().getZ(): " << laserTransform.getOrigin().getZ() << std::endl;
            srv.request.motion_type = tm_msgs::SetPositions::Request::PTP_T;
            srv.request.positions[0] = laserTransform.getOrigin().getX();
            srv.request.positions[1] = laserTransform.getOrigin().getY();
            srv.request.positions[2] = laserTransform.getOrigin().getZ();
            srv.request.positions[3] = laserTransform.getRotation().getX();
            srv.request.positions[4] = laserTransform.getRotation().getY();
            srv.request.positions[5] = laserTransform.getRotation().getZ();
            if (client.call(srv))                             
            {
                if (srv.response.ok) ROS_INFO_STREAM("SetPositions to robot");
                else ROS_WARN_STREAM("SetPositions to robot , but response not yet ok ");
            }
            else
            {
                ROS_ERROR_STREAM("Error SetPositions to robot");
                return 1;
            }
            break;
        } 
        loop_rate.sleep();   
    }
    
    // srv.request.motion_type = tm_msgs::SetPositions::Request::PTP_T;
    // srv.request.positions.push_back(laserTransform.getOrigin().getX()-gripper_offset);
    // srv.request.positions.push_back(laserTransform.getOrigin().getY());
    // srv.request.positions.push_back(laserTransform.getOrigin().getZ());
    // srv.request.positions.push_back(laserTransform.getRotation().getX());
    // srv.request.positions.push_back(laserTransform.getRotation().getY());
    // srv.request.positions.push_back(laserTransform.getRotation().getZ());
    // srv.request.velocity = 0.5;//rad/s
    // srv.request.acc_time = 0.2;
    // srv.request.blend_percentage = 10;
    // srv.request.fine_goal  = false;

    // ROS_INFO_STREAM_NAMED("SetPositions", "shutdown.");  	
    ROS_INFO("I see mailbox!");
    return 0;
    // tm_msgs::SetPositions mailbox_w;
    // mailbox_w.motion_type = 2;
    // mailbox_w.velocity = 0.2;//rad/s
    // mailbox_w.acc_time = 0.2;
    // mailbox_w.blend_percentage = 10;
    // mailbox_w.fine_goal  = false;
    // mailbox_w.positions.push_back(laserTransform.getOrigin().getX());
    // mailbox_w.positions.push_back(laserTransform.getOrigin().getY());
    // mailbox_w.positions.push_back(laserTransform.getOrigin().getZ());
    // mailbox_w.positions.push_back(laserTransform.getOrigin().getRow());
    // mailbox_w.positions.push_back(laserTransform.getOrigin().getPitch());
    // mailbox_w.positions.push_back(laserTransform.getOrigin().getYaw());

    // mailbox_pub.publish(mailbox_w);
    // std::cout << "pulishedddddddddd" << std::endl;
    
}
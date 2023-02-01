#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>


int main(int argc,char** argv)
{
    ros::init(argc,argv,"arm_listener");       //初始化ROS节点与节点名称
    ros::NodeHandle n;                         //创建节点的句柄
    ros::Rate loop_rate(10);                   //控制节点运行的频率,与loop.sleep共同使用

    //创建一个监听器，监听所有tf变换，缓冲10s
    tf::TransformListener listener;             


     while (n.ok())
    {
       
        if (listener.waitForTransform("base","mailbox",ros::Time(0),ros::Duration(10))) //等待10s，如果10s之后都还没收到消息，那么之前的消息就被丢弃掉。
        {
            tf::StampedTransform laserTransform;
            listener.lookupTransform("base","mailbox", ros::Time(0), laserTransform);
            std::cout << "laserTransform.getOrigin().getX(): " << laserTransform.getOrigin().getX() << std::endl;
            std::cout << "laserTransform.getOrigin().getY(): " << laserTransform.getOrigin().getY() << std::endl;
            std::cout << "laserTransform.getOrigin().getZ(): " << laserTransform.getOrigin().getZ() << std::endl;
        }  
        loop_rate.sleep();
    }
}
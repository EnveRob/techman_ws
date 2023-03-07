#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
int main(int argc, char** argv)
{
    ros::init(argc, argv, "camera_broadcaster2");//初始化ROS节点与节点名称
    ros::NodeHandle n;                    //创建节点的句柄
    ros::Rate loop_rate(10);             //控制节点运行的频率,与loop.sleep共同使用

    tf::TransformBroadcaster broadcaster; //创建tf广播器
    
    tf::Transform base2laser;
    tf::Quaternion q;
    q.setRPY(0,0,-0.2);
    base2laser.setRotation(q);              //设置旋转坐标
    base2laser.setOrigin(tf::Vector3(-1,-3,-3));//设平移坐标，mailbox在camera的(1,0,0)位置

    while (n.ok())
    {
        //循环发布坐标变换，两种方式
        broadcaster.sendTransform(tf::StampedTransform(base2laser,ros::Time::now(),"gripper","mailbox"));
        //broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0, 0, 0, 0), tf::Vector3(1, 0.0, 0)),ros::Time::now(),"base_link", "base_laser"));
        loop_rate.sleep();
    }
    return 0;
}
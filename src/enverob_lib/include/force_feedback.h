#ifndef FORCE_FEEDBACK_H
#define FORCE_FEEDBACK_H

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_listener.h>
#include <moveit/move_group_interface/move_group_interface.h>

namespace force_feedback
{
    class ForceCallback
    {
    private:
        geometry_msgs::Vector3 force_value_;
        moveit::planning_interface::MoveGroupInterface::Plan plan_;
        moveit::planning_interface::MoveGroupInterface *move_group_ptr_;

    public:
        ros::NodeHandle nh_;
        ros::Subscriber shut_sub_;
        ros::Subscriber sub_;

        // Constructor
        ForceCallback(moveit::planning_interface::MoveGroupInterface &move_group,
                      moveit::planning_interface::MoveGroupInterface::Plan &my_plan) : nh_("~")
        {
            shut_sub_ = nh_.subscribe("/shutdown", 10, &ForceCallback::shutCallback, this);
            sub_ = nh_.subscribe("/force_data", 1000, &ForceCallback::forceCallback, this);
            plan_ = my_plan;
            move_group_ptr_ = &move_group;
            std::cout << "------------------------------------------------------" << std::endl;
            ROS_INFO_STREAM("Subscriber in class initialized: " << sub_.getTopic());
            std::cout << "------------------------------------------------------" << std::endl;
        }
        /*ForceCallback() 是构造函数的名称， nh_("~") 是成员变量 nh_ 的初始化列表， ~表示节点的私有命名空间。
        在ROS中，通常会为节点命名空间添加一个 ~，表示该节点是私有的，即只能从该节点内部访问该命名空间中的ROS主题、服务等。
        因此，ForceCallback() : nh_("~") 意思是在 ForceCallback 类的构造函数中，初始化节点句柄 nh_ 为节点的私有命名空间。
        这样，当在类的其他成员函数中使用该节点句柄时，它们将自动使用节点的私有命名空间。*/

        // Destructor to dealocate memory
        ~ForceCallback()
        {
            delete move_group_ptr_;
        }

        void shutCallback(const std_msgs::Bool::ConstPtr &msg);
        void forceCallback(const geometry_msgs::Vector3 &msg);
        geometry_msgs::Vector3 getForceValue() const;
    };
}

#endif
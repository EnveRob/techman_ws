#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/Vector3.h>

namespace force_feedback
{
    class ForceCallback
    {
    public:
        ros::NodeHandle nh_;
        ros::Subscriber sub_;
        ros::Publisher pub_;

        ForceCallback() : nh_("~") // constructor
        {
            sub_ = nh_.subscribe("/force_data", 1000, &ForceCallback::forceCallback, this);
            pub_ = nh_.advertise<std_msgs::UInt8>("gripper_cmd", 1000);
            ROS_INFO_STREAM("Subscriber in class initialized: " << sub_.getTopic());
        }
        /*ForceCallback() 是构造函数的名称， nh_("~") 是成员变量 nh_ 的初始化列表， ~表示节点的私有命名空间。
      在ROS中，通常会为节点命名空间添加一个 ~，表示该节点是私有的，即只能从该节点内部访问该命名空间中的ROS主题、服务等。
      因此，ForceCallback() : nh_("~") 意思是在 ForceCallback 类的构造函数中，初始化节点句柄 nh_ 为节点的私有命名空间。
      这样，当在类的其他成员函数中使用该节点句柄时，它们将自动使用节点的私有命名空间。*/

        void forceCallback(const geometry_msgs::Vector3 &msg);
        geometry_msgs::Vector3 getForceValue() const;

    private:
        geometry_msgs::Vector3 force_value_;
    };
}
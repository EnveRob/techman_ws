#include <ros/ros.h>
// #include "/media/psf/EnveRob/techman_ws/src/enverob_lib/include/arm_move.h"
#include "arm_move.h"

int main(int argc, char **argv)
{
    // ------------------------ 初始化程式碼 ------------------------
    ros::init(argc, argv, "enverob_main");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // 创建一个监听器，监听所有tf变换，缓冲10s
    tf::TransformListener listener;
    tf::StampedTransform targetTransform;

    // 创建MoveGroupInterface对象，用于规划和控制机械臂
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    move_group.setGoalTolerance(0.005);
    move_group.setEndEffectorLink(END_EFFECTOR_LINK);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    // ------------------------ 手臂往信箱移動 ------------------------
    // 將機械手臂移動到初始位置
    std::vector<double> target_joint{-M_PI, -M_PI / 3, M_PI / 3 * 2, -M_PI / 3, M_PI / 2, -M_PI / 2};
    arm_move::setJointangle(move_group, my_plan, target_joint);

    // 旋轉機械手臂，尋找信箱
    double rotation_step = 0.1745;
    double current_direction = target_joint[0] + rotation_step;
    bool find_mailbox = 0;
    while (node_handle.ok() && current_direction <= 0)
    {
        target_joint[0] = current_direction;
        arm_move::setJointangle(move_group, my_plan, target_joint);
        if (listener.waitForTransform("base", "mailbox", ros::Time(0), ros::Duration(3)))
        {
            listener.lookupTransform("base", "mailbox", ros::Time(0), targetTransform);
            std::cout << "targetTransform.getOrigin().getX(): " << targetTransform.getOrigin().getX() << std::endl;
            std::cout << "targetTransform.getOrigin().getY(): " << targetTransform.getOrigin().getY() << std::endl;
            std::cout << "targetTransform.getOrigin().getZ(): " << targetTransform.getOrigin().getZ() << std::endl;
            std::cout << "targetTransform.getRotation().getX(): " << targetTransform.getRotation().getX() << std::endl;
            std::cout << "targetTransform.getRotation().getY(): " << targetTransform.getRotation().getY() << std::endl;
            std::cout << "targetTransform.getRotation().getZ(): " << targetTransform.getRotation().getZ() << std::endl;
            std::cout << "targetTransform.getRotation().getW(): " << targetTransform.getRotation().getW() << std::endl;
            find_mailbox = 1;
            break;
        }
        current_direction += rotation_step;
    }

    if (find_mailbox)
    {
        arm_move::setTargetPosition(move_group, my_plan, targetTransform);
    }
    else
    {
        std::cout << "Can't find mailbox!" << std::endl;
    }

    return 0;
}
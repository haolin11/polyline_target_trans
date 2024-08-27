#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <cmath>

// 全局变量，用于存储最终降落点、中间过渡点和当前飞机位置
geometry_msgs::PoseStamped final_landing_pose;
geometry_msgs::PoseStamped mid_point1;
geometry_msgs::PoseStamped mid_point2;

// geometry_msgs::PoseStamped current_plane_pose;

// 标志变量
// 用于控制是否结束发布第一个过渡点
bool has_arrived_first_point = false;
// 用于控制是否结束发布第二个过渡点
bool has_arrived_second_point = false;

// 函数声明
void landingPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
// void planePoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
bool isNear(const geometry_msgs::PoseStamped& pose1, double threshold);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "landing_path_planner");
    ros::NodeHandle nh;

    // 订阅降落点ENU偏置信息

    ros::Subscriber landing_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("uav1/rtk_land_pos", 10, landingPoseCallback);

    // 订阅飞机当前位置信息
    // ros::Subscriber plane_pose_sub = nh.subscribe("/mavros/local_position/pose", 10, planePoseCallback);

    // 发布目标位置ENU偏置信息
    ros::Publisher target_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("uav1/rtk_land_pos/polyline_target", 10);

    // 设置循环频率
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        if (!has_arrived_first_point)
        {
            // 发布第二个降落点作为初始目标位置
            target_pose_pub.publish(mid_point1);
            ROS_INFO_ONCE("Publishing the first mid point.");
            if (isNear(mid_point1, 0.2))
            {
                has_arrived_first_point = true;
            }
        }
        else if (!isNear(mid_point2, 0.2) && !has_arrived_second_point)
        {
            // 发布第一个降落点位置信息
            target_pose_pub.publish(mid_point1);
            ROS_INFO_ONCE("Publishing the second mid point.");
            if (isNear(mid_point1, 0.2))
            {
                has_arrived_second_point = true;
            }
        }
        else
        {
            // 发布最终降落点位置信息
            target_pose_pub.publish(final_landing_pose);
            ROS_INFO_ONCE("Publishing the final landing pose.");
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

void landingPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    final_landing_pose = *msg;

    // 计算两个中间过渡点
    mid_point1 = final_landing_pose;
    mid_point1.pose.position.x += 5;
    mid_point1.pose.position.y += 5;
    mid_point1.pose.position.z += 5;

    mid_point2 = final_landing_pose;
    mid_point2.pose.position.x += 1;
    mid_point2.pose.position.y += 0;
    mid_point2.pose.position.z += 3;

    // 设置标志，表示已发布第二个过渡点
    // has_arrived_first_point = true;
}

// void planePoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
// {
//     current_plane_pose = *msg;
// }

bool isNear(const geometry_msgs::PoseStamped& pose1, double threshold)
{
    double dx = pose1.pose.position.x;
    double dy = pose1.pose.position.y;
    double dz = pose1.pose.position.z;

    return sqrt(dx * dx + dy * dy + dz * dz) < threshold;
}
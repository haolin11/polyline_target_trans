#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <cmath>

// 全局变量，用于存储最终降落点、中间过渡点和当前飞机位置
geometry_msgs::PoseStamped final_landing_pose;
geometry_msgs::PoseStamped mid_point1;
geometry_msgs::PoseStamped mid_point2;
geometry_msgs::PoseStamped uav_current_position;

// geometry_msgs::PoseStamped current_plane_pose;

// 标志变量
// 用于控制是否结束发布第一个过渡点
bool has_arrived_first_point = false;
// 用于控制是否结束发布第二个过渡点
bool has_arrived_second_point = false;

// 函数声明
void landingPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
// void planePoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
void px4_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
bool isNear(const geometry_msgs::PoseStamped& pose1,  const geometry_msgs::PoseStamped& pose2, double threshold);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "landing_path_planner");
    ros::NodeHandle nh;

    // 订阅降落点ENU偏置信息

    //测试用submodule，并更改文件夹名称是否可以成功提交

    ros::Subscriber landing_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("uav1/rtk_land_pos", 10, landingPoseCallback);

    // 订阅飞机当前位置信息
    // ros::Subscriber plane_pose_sub = nh.subscribe("/mavros/local_position/pose", 10, planePoseCallback);
    // 【订阅】无人机当前位置 坐标系:ENU系  - 来自飞控
    // 【备注】所有状态量在飞控中均为NED系，但在ros中mavros将其转换为ENU系处理。所以，在ROS中，所有和mavros交互的量都为ENU系
    ros::Subscriber px4_position_sub = nh.subscribe<geometry_msgs::PoseStamped>("uav1/mavros/local_position/pose", 1, px4_pos_cb);

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
            if (isNear(mid_point1, uav_current_position, 0.2))
            {
                has_arrived_first_point = true;
            }
        }
        else if (!isNear(mid_point2, uav_current_position, 0.2) && !has_arrived_second_point)
        {
            // 发布第一个降落点位置信息
            target_pose_pub.publish(mid_point1);
            ROS_INFO_ONCE("Publishing the second mid point.");
            if (isNear(mid_point1, uav_current_position, 0.2))
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
void px4_pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    // 统一坐标系（RTK情况下，可以设置offset_pose，其他情况offset_pose为零）
    uav_current_position.pose.position.x = msg->pose.position.x;
    uav_current_position.pose.position.x = msg->pose.position.y;
    uav_current_position.pose.position.x = msg->pose.position.z;
}


bool isNear(const geometry_msgs::PoseStamped& pose1, const geometry_msgs::PoseStamped& pose2, double threshold)
{
    double dx = pose1.pose.position.x - pose2.pose.position.x;
    double dy = pose1.pose.position.y - pose2.pose.position.y;
    double dz = pose1.pose.position.z - pose2.pose.position.z;

    return sqrt(dx * dx + dy * dy + dz * dz) < threshold;
}
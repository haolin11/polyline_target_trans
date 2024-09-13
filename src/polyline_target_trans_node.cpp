#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <cmath>



// 定义一个结构体来表示一个点
struct linePoint {
    double x;
    double y;
};


// 全局变量，用于存储最终降落点、中间过渡点和当前飞机位置
geometry_msgs::PoseStamped final_landing_pose;
geometry_msgs::PoseStamped mid_point1;
geometry_msgs::PoseStamped mid_point2;
geometry_msgs::PoseStamped mid_point3;
geometry_msgs::PoseStamped uav_current_position;

static int count_P2 = 0;
static int count_P3 = 0;

// geometry_msgs::PoseStamped current_plane_pose;
// 过渡点的xy距降落点的距离
double xy2LandPointDistance = 4.5;

// 标志变量
// 用于控制是否结束发布第一个过渡点
bool has_arrived_first_point = false;
// 用于控制是否结束发布第二个过渡点
bool has_arrived_second_point = false;
// 用于控制是否结束发布第三个过渡点
bool has_arrived_third_point = false;

// 函数声明
void landingPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
// void planePoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
void px4_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
bool isNear(const geometry_msgs::PoseStamped& pose1,  const geometry_msgs::PoseStamped& pose2, double threshold);
linePoint findPointOnAB(double xA, double yA, double xB, double yB, double distance);


int main(int argc, char **argv)
{
    ros::init(argc, argv, "polyline_target_trans_node");
    ros::NodeHandle nh;

    // 订阅降落点ENU偏置信息

    //测试用submodule，并更改文件夹名称是否可以成功提交

    //TODO: 确认三个话题的名称
    ros::Subscriber landing_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/rtk_land_pos", 10, landingPoseCallback);

    // 订阅飞机当前位置信息
    // ros::Subscriber plane_pose_sub = nh.subscribe("/mavros/local_position/pose", 10, planePoseCallback);
    // 【订阅】无人机当前位置 坐标系:ENU系  - 来自飞控
    // 【备注】所有状态量在飞控中均为NED系，但在ros中mavros将其转换为ENU系处理。所以，在ROS中，所有和mavros交互的量都为ENU系
    ros::Subscriber px4_position_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1, px4_pos_cb);

    // 发布目标位置ENU偏置信息
    ros::Publisher target_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/rtk_land_pos/polyline_target", 10);

    // 设置循环频率
    ros::Rate loop_rate(10);
    // // 程序暂停1秒，等待数值稳定
    // ros::Duration(1).sleep();

    while (ros::ok())
    {
        if(mid_point1.pose.position.x == mid_point1.pose.position.y == 0)
        if (!has_arrived_first_point)
        {
            // 发布第一个降落点作为初始目标位置
            target_pose_pub.publish(mid_point1);
            ROS_INFO_ONCE("Publishing the first mid point.");
            if (isNear(mid_point1, uav_current_position, 0.4))
            {
                // std::cout<< "接近第一个点了！"<< std::endl;
                has_arrived_first_point = true;
            }
        }
        else if ( !has_arrived_second_point)
        {
            // 发布第二个降落点位置信息
            target_pose_pub.publish(mid_point2);
            ROS_INFO_ONCE("Publishing the second mid point.");
            if (isNear(mid_point2, uav_current_position, 0.4))
            {
                count_P2++;
                if (count_P2 == 10)
                {
                    has_arrived_second_point = true;
                }
                
            }
        }
        else if ( !has_arrived_third_point)
        {
            // 发布第二个降落点位置信息
            target_pose_pub.publish(mid_point3);
            ROS_INFO_ONCE("Publishing the third mid point.");
            if (isNear(mid_point3, uav_current_position, 0.4))
            {
                count_P3++;
                if (count_P3 == 10)
                {
                    has_arrived_third_point = true;
                }
                
            }
        }
        else
        {
            // 发布最终降落点位置信息
            // final_landing_pose.pose.position.z +=1.0;
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
    // 靠近当前无人机的为第一个过渡点，第一个点采用连线的方式取点
    linePoint mid_point1_xy = findPointOnAB(uav_current_position.pose.position.x, uav_current_position.pose.position.y,
           final_landing_pose.pose.position.x, final_landing_pose.pose.position.y, xy2LandPointDistance );

    // mid_point1 = final_landing_pose;
    mid_point1.pose.position.x = mid_point1_xy.x;
    mid_point1.pose.position.y = mid_point1_xy.y;
    mid_point1.pose.position.z = final_landing_pose.pose.position.z + 3.0;

    //第二个点与第一个点高度保持一致
    mid_point2 = final_landing_pose;
    mid_point2.pose.position.x += 0;
    mid_point2.pose.position.y += 0;
    mid_point2.pose.position.z += 3.0;

    //第三个点与第二个点xy保持一致
    mid_point3 = final_landing_pose;
    mid_point3.pose.position.x += 0;
    mid_point3.pose.position.y += 0;
    mid_point3.pose.position.z += 1.5;

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
    uav_current_position.pose.position.y = msg->pose.position.y;
    uav_current_position.pose.position.z = msg->pose.position.z;
}


bool isNear(const geometry_msgs::PoseStamped& pose1, const geometry_msgs::PoseStamped& pose2, double threshold)
{
    double dx = pose1.pose.position.x - pose2.pose.position.x;
    double dy = pose1.pose.position.y - pose2.pose.position.y;
    double dz = pose1.pose.position.z - pose2.pose.position.z;
    // std::cout<<"pose1"<<pose1<<std::endl;
    // std::cout<<"pose2"<<pose2<<std::endl;
    // std::cout<<"距离值："<<sqrt(dx * dx + dy * dy + dz * dz)<<std::endl;

    return sqrt(dx * dx + dy * dy + dz * dz) < threshold;
}


// 函数用于计算线段AB上距离B点指定距离的坐标
linePoint findPointOnAB(double xA, double yA, double xB, double yB, double distance) {
    // 计算向量AB的坐标差
    double deltaX = xB - xA;
    double deltaY = yB - yA;

    // 计算向量AB的长度
    double lengthAB = sqrt(deltaX * deltaX + deltaY * deltaY);

    // 计算单位向量
    double unitX = deltaX / lengthAB;
    double unitY = deltaY / lengthAB;

    // 计算点C的坐标，沿单位向量反方向移动distance米
    linePoint pointC;
    pointC.x = xB - distance * unitX;
    pointC.y = yB - distance * unitY;

    // 返回点C
    return pointC;
}

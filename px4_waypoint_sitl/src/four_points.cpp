#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <vector>
#include <cmath>

// PX4 + MAVROS + ROS1 + Gazebo Classic SITL四点仿真
mavros_msgs::State state;
/*
bool connected
bool armed
bool guided
bool manual_input
string mode
unit8 system_status
*/
geometry_msgs::PoseStamped current_pose;
/*
geometry_msgs/Pose pose
geometry_msgs/Point position
float64 x
float64 y
float64 z
*/
bool pose_received = false;

// 状态回调
void stateCallback(const mavros_msgs::State::ConstPtr &msg){
    state = *msg;
}

// 姿态回调
void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg){
    current_pose = *msg;
    pose_received = true;
}

double distanceTo(const geometry_msgs::PoseStamped &target){
    auto a = current_pose.pose.position; // geometry_msgs::Point
    auto b = target.pose.position;
    // 三维欧式距离
    return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2) + std::pow(a.z - b.z, 2));
}

int main(int argc, char** argv){
    ros::init(argc, argv, "four_points");
    ros::NodeHandle nh; // 句柄
    // 状态订阅
    auto state_sub = nh.subscribe("/mavros/state", 10, stateCallback); //ros::Publisher

    //姿态订阅
    auto pose_sub = nh.subscribe("/mavros/local_position/pose", 10, poseCallback);

    // 声明发布
    auto pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);

    // OFFBOARD / 解锁服务
    auto setmode_cl = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    auto arming_cl = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");

    ros::Rate rate(20); // 主循环频率

    while (ros::ok() && (!state.connected || !pose_received)){
        ros::spinOnce(); // 阻塞
        rate.sleep();
    }

    if (!ros::ok()) return 0;

    auto origin = current_pose;
    std::vector<geometry_msgs::PoseStamped> points;

    // 四点正方形
    const double offsets[4][2] = {{1, 0}, {1, 1}, {0, 1}, {0, 0}};

    for (const auto& offset: offsets){
        auto p = origin;
        p.pose.position.x += offset[0];
        p.pose.position.y += offset[1];

        points.push_back(p);
    }

    // 先持续发送第一个航点，让 FCU 能接受 OFFBOARD 模式
    for (int i = 0; ros::ok() && i < 100; i++){
        auto target = points[0];
        target.header.stamp = ros::Time::now();
        target.header.frame_id = "map";
        pub.publish(target);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offboard_mode;
    offboard_mode.request.custom_mode = "OFFBOARD";
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    ros::Time last_request = ros::Time::now();

    size_t index = 0;
    ros::Time reached_since; // 第一次进入某航点误差范围时间
    ros::Time waypoint_start = ros::Time::now(); // 开始执行某航点时间

    while (ros::ok()){
        ros::spinOnce(); // 阻塞等待

        if (!state.connected || !pose_received) break;

        // 请求 OFFBOARD，成功后再解锁
        if (state.mode != "OFFBOARD" && (ros::Time::now() - last_request) > ros::Duration(5.0)){
            if (setmode_cl.call(offboard_mode) && offboard_mode.response.mode_sent){
                ROS_INFO("OFFBOARD enabled");
            }
            else{
                ROS_INFO("OFFBOARD set mode failed");
            }
            last_request = ros::Time::now();
        }
        else if (!state.armed && (ros::Time::now() - last_request) > ros::Duration(5.0)){
            if (arming_cl.call(arm_cmd) && arm_cmd.response.success){
                ROS_INFO("Vehicle armed");
            }
            else{
                ROS_INFO("Vehicle arm failed");
            }
            last_request = ros::Time::now();
        }

        auto target = points[index];
        target.header.stamp = ros::Time::now();
        target.header.frame_id = "map";
        pub.publish(target);

        if (distanceTo(target) < 0.3){ // 待实际测试调试
            if (reached_since.isZero()) reached_since = ros::Time::now();

            if ((ros::Time::now() - reached_since).toSec() > 2.0){ // 秒转换 待测试
                if (index + 1 < points.size()){
                    ++index;
                    reached_since = ros::Time();
                    waypoint_start = ros::Time::now();
                    ROS_INFO("Waypoint %zu", index + 1);
                }
                else{
                    ROS_INFO("All waypoints reached.");
                    break;
                }
            }
        }
        else{
            reached_since = ros::Time();
        }

        if((ros::Time::now() - waypoint_start).toSec() > 30.0){ // 待测试
            ROS_ERROR("Waypoint timeout.");
            break;
        }
        rate.sleep();
    }
    return 0;
}
#ifndef RM_CONTROL_H
#define RM_CONTROL_H

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <iostream>
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
// #include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/u_int32.hpp>

//RM Robot msg
#include "rm_ros_interfaces/msg/jointpos.hpp"
//#include "rm_ros_interfaces/msg/jointpos75.hpp"

/* 使用变长数组 */
#include <vector>
#include <algorithm>
#include <mutex>

using namespace std;

class Rm_Control : public rclcpp::Node
{
public:
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    using GoalHandleFJT = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

    explicit Rm_Control(std::string name);
    ~Rm_Control(){}

    void timer_callback();

private:
    rm_ros_interfaces::msg::Jointpos joint_msg;
    // rm_ros_interfaces::msg::Jointpos75 joint7_msg;
    int arm_type_ = 75;
    bool follow_ = true;
    // 实例化样条
    rclcpp_action::Server<FollowJointTrajectory>::SharedPtr action_server_;

    // 声明话题发布者
    rclcpp::Publisher<rm_ros_interfaces::msg::Jointpos>::SharedPtr joint_pos_publisher;
    // rclcpp::Publisher<rm_ros_interfaces::msg::Jointpos75>::SharedPtr joint_pos_publisher_75;

    //声明话题订阅者
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr Get_Move_Stop_Cmd;
    rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr Movej_CANFD_Result;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr Joint_State_Sub;

    std::mutex joint_state_mutex_;
    std::vector<std::string> latest_joint_names_;
    std::vector<double> latest_joint_positions_;
    bool joint_state_received_ = false;

    rclcpp::TimerBase::SharedPtr State_Timer;

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const FollowJointTrajectory::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleFJT> goal_handle);
    void execute_move(const std::shared_ptr<GoalHandleFJT> goal_handle);
    void handle_accepted(const std::shared_ptr<GoalHandleFJT> goal_handle);
    void get_move_stop_callback(std_msgs::msg::Empty::SharedPtr msg);
    void movej_canfd_result_callback(std_msgs::msg::UInt32::SharedPtr msg);
    void joint_state_callback(sensor_msgs::msg::JointState::SharedPtr msg);
};

#endif // Rm_Control_H


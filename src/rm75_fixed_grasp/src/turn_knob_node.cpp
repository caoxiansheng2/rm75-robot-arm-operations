#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <future>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rm_ros_interfaces/msg/movejp.hpp>
#include <rm_ros_interfaces/msg/movel.hpp>
#include <rm_ros_interfaces/msg/modbusrtuwriteparams.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

class TurnKnobNode : public rclcpp::Node
{
public:
  TurnKnobNode()
  : Node("rm75_turn_knob"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    eef_frame_ = declare_parameter<std::string>("end_effector_link", "Link7");
    target_frame_ = declare_parameter<std::string>("target_frame", "knob_tag");
    offset_ = declare_parameter<std::vector<double>>(
      "target_offset_base_xyz", {0.0, 0.0, 0.0});
    precontact_distance_ = declare_parameter<double>("precontact_distance", 0.20);
    contact_standoff_ = declare_parameter<double>("contact_standoff", 0.14);
    max_precontact_translation_ = declare_parameter<double>(
      "maximum_precontact_translation", 0.50);
    max_contact_translation_ = declare_parameter<double>(
      "maximum_contact_translation", 0.15);
    tf_timeout_ = declare_parameter<double>("tf_timeout_seconds", 1.0);
    target_max_age_ = declare_parameter<double>("target_max_age_seconds", 0.75);

    pre_rotate_deg_ = declare_parameter<double>("knob_pre_rotate_degrees", -10.0);
    turn_deg_ = declare_parameter<double>("knob_turn_degrees", 20.0);
    max_rotate_deg_ = declare_parameter<double>(
      "knob_max_abs_rotation_degrees", 90.0);
    return_joints_ = declare_parameter<std::vector<double>>(
      "return_joint_values",
      {-1.4836688362121582, 1.2940047286987304, 0.001291300016641617,
        0.8802303638458252, -0.04767340196371079, -2.190864993667603,
        0.6988724866867065});

    speed_ = declare_parameter<int>("driver_speed", 10);
    driver_timeout_ = declare_parameter<double>("driver_result_timeout_seconds", 90.0);
    joint_timeout_ = declare_parameter<double>("joint_motion_timeout_seconds", 90.0);
    joint_tolerance_ = declare_parameter<double>("joint_position_tolerance", 0.03);
    joint_trajectory_duration_ = declare_parameter<double>(
      "joint_trajectory_duration_seconds", 4.0);
    joint_trajectory_waypoints_ = declare_parameter<int>(
      "joint_trajectory_waypoints", 5);
    gripper_address_ = declare_parameter<int>("gripper_register_address", 40000);
    gripper_device_ = declare_parameter<int>("gripper_device", 1);
    gripper_type_ = declare_parameter<int>("gripper_modbus_type", 1);
    gripper_open_ = declare_parameter<int>("gripper_open_value", 100);
    gripper_close_ = declare_parameter<int>("gripper_close_value", 0);
    gripper_wait_ = declare_parameter<double>("gripper_wait_seconds", 1.5);
    allow_motion_ = declare_parameter<bool>("allow_motion", false);
    dry_run_ = declare_parameter<bool>("dry_run", true);
    validate();

    movejp_pub_ = create_publisher<rm_ros_interfaces::msg::Movejp>(
      "/rm_driver/movej_p_cmd", rclcpp::QoS(10).reliable());
    movel_pub_ = create_publisher<rm_ros_interfaces::msg::Movel>(
      "/rm_driver/movel_cmd", rclcpp::QoS(10).reliable());
    trajectory_client_ = rclcpp_action::create_client<FollowTrajectory>(
      this, "/rm_group_controller/follow_joint_trajectory");
    gripper_pub_ = create_publisher<rm_ros_interfaces::msg::Modbusrtuwriteparams>(
      "/rm_driver/write_modbus_rtu_registers_cmd", rclcpp::QoS(10).reliable());

    movejp_result_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/rm_driver/movej_p_result", rclcpp::QoS(10).reliable(),
      [this](std_msgs::msg::Bool::SharedPtr msg) {
        {
          std::lock_guard<std::mutex> lock(result_mutex_);
          movejp_result_ = msg->data;
          movejp_received_ = true;
        }
        result_cv_.notify_all();
      });
    movel_result_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/rm_driver/movel_result", rclcpp::QoS(10).reliable(),
      [this](std_msgs::msg::Bool::SharedPtr msg) {
        {
          std::lock_guard<std::mutex> lock(result_mutex_);
          movel_result_ = msg->data;
          movel_received_ = true;
        }
        result_cv_.notify_all();
      });
    arm_pose_sub_ = create_subscription<geometry_msgs::msg::Pose>(
      "/rm_driver/udp_arm_position", rclcpp::QoS(10).best_effort(),
      [this](geometry_msgs::msg::Pose::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        arm_pose_ = *msg;
        have_arm_pose_ = true;
      });
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::QoS(20).best_effort(),
      [this](sensor_msgs::msg::JointState::SharedPtr msg) {
        std::vector<double> ordered(7);
        for (std::size_t i = 0; i < 7; ++i) {
          const auto name = "joint" + std::to_string(i + 1);
          const auto it = std::find(msg->name.begin(), msg->name.end(), name);
          if (it == msg->name.end()) {return;}
          const auto index = static_cast<std::size_t>(std::distance(msg->name.begin(), it));
          if (index >= msg->position.size()) {return;}
          ordered[i] = msg->position[index];
        }
        {
          std::lock_guard<std::mutex> lock(state_mutex_);
          joints_ = ordered;
          have_joints_ = true;
        }
        state_cv_.notify_all();
      });

    callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    execute_service_ = create_service<std_srvs::srv::Trigger>(
      "~/execute",
      std::bind(&TurnKnobNode::execute, this, std::placeholders::_1, std::placeholders::_2),
      rclcpp::ServicesQoS().get_rmw_qos_profile(), callback_group_);
    RCLCPP_INFO(
      get_logger(), "Ready. tag=%s pre-rotate=%.1f deg turn=%.1f deg",
      target_frame_.c_str(), pre_rotate_deg_, turn_deg_);
  }

private:
  struct Targets
  {
    geometry_msgs::msg::Pose precontact;
    geometry_msgs::msg::Pose contact;
  };

  static geometry_msgs::msg::Point point(const tf2::Vector3 & value)
  {
    geometry_msgs::msg::Point result;
    result.x = value.x(); result.y = value.y(); result.z = value.z();
    return result;
  }

  static double distance(
    const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b)
  {
    return std::hypot(std::hypot(a.x - b.x, a.y - b.y), a.z - b.z);
  }

  void validate() const
  {
    if (offset_.size() != 3 || return_joints_.size() != 7) {
      throw std::runtime_error("target offset needs 3 values and return joints need 7");
    }
    if (precontact_distance_ <= contact_standoff_ || contact_standoff_ < 0.0) {
      throw std::runtime_error("precontact_distance must exceed contact_standoff");
    }
    if (std::abs(pre_rotate_deg_) > max_rotate_deg_ ||
      std::abs(turn_deg_) > max_rotate_deg_ || max_rotate_deg_ <= 0.0)
    {
      throw std::runtime_error("joint7 rotation exceeds configured safety limit");
    }
    if (max_precontact_translation_ <= 0.0 || max_contact_translation_ <= 0.0 ||
      tf_timeout_ <= 0.0 || target_max_age_ <= 0.0 || speed_ <= 0 || speed_ > 100 ||
      driver_timeout_ <= 0.0 || joint_timeout_ <= 0.0 || joint_tolerance_ <= 0.0 ||
      joint_trajectory_duration_ <= 0.0 || joint_trajectory_waypoints_ < 4 ||
      gripper_wait_ < 0.0)
    {
      throw std::runtime_error("invalid positive motion/safety parameter");
    }
  }

  Targets computeTargets()
  {
    const auto tag = tf_buffer_.lookupTransform(
      base_frame_, target_frame_, tf2::TimePointZero, tf2::durationFromSec(tf_timeout_));
    const rclcpp::Time stamp(tag.header.stamp, get_clock()->get_clock_type());
    const double age = (now() - stamp).seconds();
    if (age < -0.05 || age > target_max_age_) {
      throw std::runtime_error("knob_tag TF is stale; age=" + std::to_string(age));
    }
    const auto link = tf_buffer_.lookupTransform(
      base_frame_, eef_frame_, tf2::TimePointZero, tf2::durationFromSec(tf_timeout_));
    tf2::Transform base_tag;
    tf2::fromMsg(tag.transform, base_tag);
    const tf2::Vector3 tag_position = base_tag.getOrigin();
    const tf2::Vector3 link_position(
      link.transform.translation.x, link.transform.translation.y,
      link.transform.translation.z);
    tf2::Vector3 normal = tf2::quatRotate(
      base_tag.getRotation(), tf2::Vector3(0.0, 0.0, 1.0));
    normal.normalize();
    if (normal.dot(link_position - tag_position) < 0.0) {normal *= -1.0;}
    const tf2::Vector3 offset(offset_[0], offset_[1], offset_[2]);

    Targets targets;
    targets.precontact.position = point(tag_position + offset + normal * precontact_distance_);
    targets.contact.position = point(tag_position + offset + normal * contact_standoff_);
    targets.precontact.orientation = link.transform.rotation;
    targets.contact.orientation = link.transform.rotation;
    const double to_precontact = distance(point(link_position), targets.precontact.position);
    const double approach = distance(targets.precontact.position, targets.contact.position);
    if (to_precontact > max_precontact_translation_) {
      throw std::runtime_error("precontact translation exceeds safety limit: " +
              std::to_string(to_precontact));
    }
    if (approach > max_contact_translation_) {
      throw std::runtime_error("contact translation exceeds safety limit: " +
              std::to_string(approach));
    }
    RCLCPP_INFO(
      get_logger(), "precontact=[%.4f %.4f %.4f], contact=[%.4f %.4f %.4f]",
      targets.precontact.position.x, targets.precontact.position.y,
      targets.precontact.position.z, targets.contact.position.x,
      targets.contact.position.y, targets.contact.position.z);
    return targets;
  }

  geometry_msgs::msg::Pose driverTarget(const geometry_msgs::msg::Pose & link_target)
  {
    geometry_msgs::msg::Pose target;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (!have_arm_pose_) {throw std::runtime_error("no controller arm pose received");}
      target = arm_pose_;
    }
    const auto link = tf_buffer_.lookupTransform(
      base_frame_, eef_frame_, tf2::TimePointZero, tf2::durationFromSec(tf_timeout_));
    target.position.x += link_target.position.x - link.transform.translation.x;
    target.position.y += link_target.position.y - link.transform.translation.y;
    target.position.z += link_target.position.z - link.transform.translation.z;
    return target;
  }

  bool sendMoveJP(const geometry_msgs::msg::Pose & target)
  {
    {
      std::lock_guard<std::mutex> lock(result_mutex_);
      movejp_received_ = false;
    }
    rm_ros_interfaces::msg::Movejp msg;
    msg.pose = driverTarget(target); msg.speed = speed_; msg.block = true;
    msg.trajectory_connect = 0; movejp_pub_->publish(msg);
    std::unique_lock<std::mutex> lock(result_mutex_);
    return result_cv_.wait_for(
      lock, std::chrono::duration<double>(driver_timeout_),
      [this]() {return movejp_received_;}) && movejp_result_;
  }

  bool sendMoveL(const geometry_msgs::msg::Pose & target)
  {
    {
      std::lock_guard<std::mutex> lock(result_mutex_);
      movel_received_ = false;
    }
    rm_ros_interfaces::msg::Movel msg;
    msg.pose = driverTarget(target); msg.speed = speed_; msg.block = true;
    msg.trajectory_connect = 0; movel_pub_->publish(msg);
    std::unique_lock<std::mutex> lock(result_mutex_);
    return result_cv_.wait_for(
      lock, std::chrono::duration<double>(driver_timeout_),
      [this]() {return movel_received_;}) && movel_result_;
  }

  std::vector<double> currentJoints()
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!have_joints_) {throw std::runtime_error("no seven-axis joint state received");}
    return joints_;
  }

  bool sendMoveJ(const std::vector<double> & target, const std::string & label)
  {
    if (!trajectory_client_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(get_logger(), "trajectory action server is unavailable");
      return false;
    }
    FollowTrajectory::Goal goal;
    goal.trajectory.joint_names = {
      "joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"};

    // This rm_control implementation only applies time-based cubic
    // interpolation when it receives more than three points. With one or two
    // points it sends them at its 5 ms timer rate and effectively requests an
    // instantaneous joint jump. Generate a small time-parameterized path so
    // rm_control expands it into a smooth CANFD stream.
    const auto start = currentJoints();
    for (int waypoint = 0; waypoint < joint_trajectory_waypoints_; ++waypoint) {
      const double ratio = static_cast<double>(waypoint) /
        static_cast<double>(joint_trajectory_waypoints_ - 1);
      trajectory_msgs::msg::JointTrajectoryPoint point;
      point.positions.resize(target.size());
      for (std::size_t joint = 0; joint < target.size(); ++joint) {
        point.positions[joint] = start[joint] + ratio * (target[joint] - start[joint]);
      }
      const auto waypoint_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(joint_trajectory_duration_ * ratio)).count();
      point.time_from_start.sec = static_cast<int32_t>(waypoint_ns / 1000000000LL);
      point.time_from_start.nanosec = static_cast<uint32_t>(waypoint_ns % 1000000000LL);
      goal.trajectory.points.push_back(point);
    }
    auto goal_future = trajectory_client_->async_send_goal(goal);
    if (goal_future.wait_for(std::chrono::seconds(10)) != std::future_status::ready) {
      RCLCPP_ERROR(get_logger(), "%s goal acceptance timed out", label.c_str());
      return false;
    }
    const auto goal_handle = goal_future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(get_logger(), "%s goal was rejected", label.c_str());
      return false;
    }
    auto result_future = trajectory_client_->async_get_result(goal_handle);
    if (result_future.wait_for(std::chrono::duration<double>(joint_timeout_)) !=
      std::future_status::ready)
    {
      RCLCPP_ERROR(get_logger(), "%s action result timed out", label.c_str());
      return false;
    }
    if (result_future.get().code != rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_ERROR(get_logger(), "%s trajectory execution failed", label.c_str());
      return false;
    }
    RCLCPP_INFO(get_logger(), "%s trajectory completed", label.c_str());
    std::unique_lock<std::mutex> lock(state_mutex_);
    const bool reached = state_cv_.wait_for(
      lock, std::chrono::duration<double>(joint_timeout_), [this, &target]() {
        if (!have_joints_ || joints_.size() != target.size()) {return false;}
        for (std::size_t i = 0; i < target.size(); ++i) {
          if (std::abs(joints_[i] - target[i]) > joint_tolerance_) {return false;}
        }
        return true;
      });
    if (!reached) {RCLCPP_ERROR(get_logger(), "%s timed out", label.c_str());}
    return reached;
  }

  bool rotateJoint7(double degrees, const std::string & label)
  {
    constexpr double pi = 3.14159265358979323846;
    auto target = currentJoints();
    target[6] += degrees * pi / 180.0;
    return sendMoveJ(target, label);
  }

  void gripper(int value, const std::string & label, bool wait)
  {
    rm_ros_interfaces::msg::Modbusrtuwriteparams msg;
    msg.address = gripper_address_; msg.device = gripper_device_;
    msg.type = gripper_type_; msg.num = 1; msg.data = {value};
    gripper_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "%s, value=%d", label.c_str(), value);
    if (wait && gripper_wait_ > 0.0) {
      rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(gripper_wait_)));
    }
  }

  void runSequence()
  {
    if (!allow_motion_ || dry_run_) {
      throw std::runtime_error("motion locked: require allow_motion=true and dry_run=false");
    }
    const auto targets = computeTargets();
    if (!sendMoveJP(targets.precontact)) {throw std::runtime_error("precontact failed");}
    if (!rotateJoint7(pre_rotate_deg_, "joint7 pre-rotation")) {
      throw std::runtime_error("joint7 pre-rotation failed");
    }
    gripper(gripper_open_, "open gripper", true);
    if (!sendMoveL(targets.contact)) {throw std::runtime_error("knob approach failed");}
    gripper(gripper_close_, "close on knob", true);
    if (!rotateJoint7(turn_deg_, "turn knob")) {throw std::runtime_error("knob turn failed");}
    // Release the knob first and keep the gripper open during retreat.
    gripper(gripper_open_, "release knob", true);

    if (!sendMoveL(targets.precontact)) {
      throw std::runtime_error("retreat failed");
    }

    // Close only after Link7 has completely returned to precontact.
    gripper(gripper_close_, "close after reaching precontact", true);
        if (!sendMoveJ(return_joints_, "return fixed pose")) {
          throw std::runtime_error("fixed-pose return failed");
        }
      }

  void execute(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    if (busy_.exchange(true)) {
      response->message = "another knob request is running"; return;
    }
    try {
      runSequence(); response->success = true;
      response->message = "knob sequence completed and robot returned to fixed pose";
    } catch (const std::exception & error) {
      response->success = false; response->message = error.what();
      RCLCPP_ERROR(get_logger(), "Knob sequence failed: %s", error.what());
    }
    busy_ = false;
  }

  std::string base_frame_, eef_frame_, target_frame_;
  std::vector<double> offset_, return_joints_, joints_;
  double precontact_distance_, contact_standoff_;
  double max_precontact_translation_, max_contact_translation_;
  double tf_timeout_, target_max_age_, pre_rotate_deg_, turn_deg_, max_rotate_deg_;
  double driver_timeout_, joint_timeout_, joint_tolerance_, joint_trajectory_duration_;
  double gripper_wait_;
  int joint_trajectory_waypoints_;
  int speed_, gripper_address_, gripper_device_, gripper_type_, gripper_open_, gripper_close_;
  bool allow_motion_, dry_run_, have_arm_pose_{false}, have_joints_{false};
  std::atomic_bool busy_{false};
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<rm_ros_interfaces::msg::Movejp>::SharedPtr movejp_pub_;
  rclcpp::Publisher<rm_ros_interfaces::msg::Movel>::SharedPtr movel_pub_;
  rclcpp::Publisher<rm_ros_interfaces::msg::Modbusrtuwriteparams>::SharedPtr gripper_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr movejp_result_sub_, movel_result_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr arm_pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr execute_service_;
  using FollowTrajectory = control_msgs::action::FollowJointTrajectory;
  rclcpp_action::Client<FollowTrajectory>::SharedPtr trajectory_client_;
  std::mutex result_mutex_, state_mutex_;
  std::condition_variable result_cv_, state_cv_;
  bool movejp_received_{false}, movejp_result_{false};
  bool movel_received_{false}, movel_result_{false};
  geometry_msgs::msg::Pose arm_pose_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TurnKnobNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}

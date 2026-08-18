#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rm_ros_interfaces/msg/movejp.hpp>
#include <rm_ros_interfaces/msg/movel.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using namespace std::chrono_literals;

class FixedGraspNode : public rclcpp::Node
{
public:
  FixedGraspNode()
  : Node("rm75_fixed_grasp"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    planning_group_ = declare_parameter<std::string>("planning_group", "rm_group");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    end_effector_link_ = declare_parameter<std::string>("end_effector_link", "Link7");
    target_frame_ = declare_parameter<std::string>("target_frame", "grab_tag");
    return_named_target_ = declare_parameter<std::string>("return_named_target", "forward");
    return_joint_values_ = declare_parameter<std::vector<double>>(
      "return_joint_values",
      {-1.4836688362121582, 1.2940047286987304, 0.001291300016641617,
        0.8802303638458252, -0.04767340196371079, -2.190864993667603,
        0.6988724866867065});
    auto_return_after_execute_ = declare_parameter<bool>(
      "auto_return_after_execute", false);

    target_offset_base_xyz_ = declare_parameter<std::vector<double>>(
      "target_offset_base_xyz", {0.0, 0.0, 0.0});
    precontact_distance_ = declare_parameter<double>("precontact_distance", 0.10);
    contact_standoff_ = declare_parameter<double>("contact_standoff", 0.05);
    execute_contact_ = declare_parameter<bool>("execute_contact", true);
    auto_retract_ = declare_parameter<bool>("auto_retract", false);
    single_stage_to_contact_ = declare_parameter<bool>("single_stage_to_contact", false);

    eef_step_ = declare_parameter<double>("eef_step", 0.005);
    jump_threshold_ = declare_parameter<double>("jump_threshold", 0.0);
    minimum_cartesian_fraction_ = declare_parameter<double>(
      "minimum_cartesian_fraction", 0.98);
    position_tolerance_ = declare_parameter<double>("position_tolerance", 0.01);
    orientation_tolerance_ = declare_parameter<double>("orientation_tolerance", 0.10);
    velocity_scale_ = declare_parameter<double>("velocity_scale", 0.05);
    acceleration_scale_ = declare_parameter<double>("acceleration_scale", 0.05);
    planning_time_seconds_ = declare_parameter<double>("planning_time_seconds", 10.0);
    planning_attempts_ = declare_parameter<int>("planning_attempts", 5);
    tf_timeout_seconds_ = declare_parameter<double>("tf_timeout_seconds", 1.0);
    target_max_age_seconds_ = declare_parameter<double>("target_max_age_seconds", 0.75);
    maximum_precontact_translation_ = declare_parameter<double>(
      "maximum_precontact_translation", 0.45);
    maximum_contact_translation_ = declare_parameter<double>(
      "maximum_contact_translation", 0.15);
    use_rm_driver_backend_ = declare_parameter<bool>("use_rm_driver_backend", true);
    driver_speed_ = declare_parameter<int>("driver_speed", 10);
    driver_result_timeout_seconds_ = declare_parameter<double>(
      "driver_result_timeout_seconds", 90.0);

    allow_motion_ = declare_parameter<bool>("allow_motion", false);
    dry_run_ = declare_parameter<bool>("dry_run", true);
    auto_execute_ = declare_parameter<bool>("auto_execute", false);
    auto_target_stability_seconds_ = declare_parameter<double>(
      "auto_target_stability_seconds", 1.5);
    auto_target_position_threshold_ = declare_parameter<double>(
      "auto_target_position_threshold", 0.01);

    validateParameters();

    movejp_publisher_ = create_publisher<rm_ros_interfaces::msg::Movejp>(
      "/rm_driver/movej_p_cmd", rclcpp::QoS(10).reliable());
    movel_publisher_ = create_publisher<rm_ros_interfaces::msg::Movel>(
      "/rm_driver/movel_cmd", rclcpp::QoS(10).reliable());
    movejp_result_subscription_ = create_subscription<std_msgs::msg::Bool>(
      "/rm_driver/movej_p_result", rclcpp::QoS(10).reliable(),
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        {
          std::lock_guard<std::mutex> lock(driver_result_mutex_);
          movejp_result_ = msg->data;
          movejp_result_received_ = true;
        }
        driver_result_cv_.notify_all();
      });
    movel_result_subscription_ = create_subscription<std_msgs::msg::Bool>(
      "/rm_driver/movel_result", rclcpp::QoS(10).reliable(),
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        {
          std::lock_guard<std::mutex> lock(driver_result_mutex_);
          movel_result_ = msg->data;
          movel_result_received_ = true;
        }
        driver_result_cv_.notify_all();
      });
    arm_pose_subscription_ = create_subscription<geometry_msgs::msg::Pose>(
      "/rm_driver/udp_arm_position", rclcpp::QoS(10).best_effort(),
      [this](const geometry_msgs::msg::Pose::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(arm_pose_mutex_);
        latest_arm_pose_ = *msg;
        arm_pose_received_ = true;
      });

    service_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    const auto qos = rclcpp::ServicesQoS().get_rmw_qos_profile();
    execute_service_ = create_service<std_srvs::srv::Trigger>(
      "~/execute",
      std::bind(
        &FixedGraspNode::executeCallback, this, std::placeholders::_1,
        std::placeholders::_2),
      qos, service_callback_group_);
    preview_service_ = create_service<std_srvs::srv::Trigger>(
      "~/preview",
      std::bind(
        &FixedGraspNode::previewCallback, this, std::placeholders::_1,
        std::placeholders::_2),
      qos, service_callback_group_);
    retract_service_ = create_service<std_srvs::srv::Trigger>(
      "~/retract",
      std::bind(
        &FixedGraspNode::retractCallback, this, std::placeholders::_1,
        std::placeholders::_2),
      qos, service_callback_group_);
    return_service_ = create_service<std_srvs::srv::Trigger>(
      "~/return_home",
      std::bind(
        &FixedGraspNode::returnCallback, this, std::placeholders::_1,
        std::placeholders::_2),
      qos, service_callback_group_);
  }

  void initializeMoveIt()
  {
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), planning_group_);
    move_group_->setPoseReferenceFrame(base_frame_);
    move_group_->setEndEffectorLink(end_effector_link_);
    move_group_->setGoalPositionTolerance(position_tolerance_);
    move_group_->setGoalOrientationTolerance(orientation_tolerance_);
    move_group_->setMaxVelocityScalingFactor(clampScale(velocity_scale_));
    move_group_->setMaxAccelerationScalingFactor(clampScale(acceleration_scale_));
    move_group_->setPlanningTime(planning_time_seconds_);
    move_group_->setNumPlanningAttempts(planning_attempts_);

    RCLCPP_INFO(
      get_logger(),
      "Ready. %s tag motion: %s. "
      "group=%s base=%s eef=%s tag=%s precontact=%.3f m standoff=%.3f m "
      "allow_motion=%s dry_run=%s",
      single_stage_to_contact_ ? "single-stage" : "two-stage",
      single_stage_to_contact_ ?
      (use_rm_driver_backend_ ? "RM MoveJ_P directly to contact" : "Pilz PTP directly to contact") :
      (use_rm_driver_backend_ ? "RM MoveJ_P to precontact, then RM MoveL" :
      "Pilz PTP to precontact, then MoveIt Cartesian"),
      planning_group_.c_str(), base_frame_.c_str(), end_effector_link_.c_str(),
      target_frame_.c_str(), precontact_distance_, contact_standoff_,
      allow_motion_ ? "true" : "false", dry_run_ ? "true" : "false");

    if (auto_execute_) {
      auto_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
      const std::string execute_service_name =
        std::string(get_fully_qualified_name()) + "/execute";
      auto_execute_client_ = create_client<std_srvs::srv::Trigger>(
        execute_service_name, rclcpp::ServicesQoS().get_rmw_qos_profile(),
        auto_callback_group_);
      auto_execute_timer_ = create_wall_timer(
        250ms, std::bind(&FixedGraspNode::autoExecuteTick, this),
        auto_callback_group_);
      RCLCPP_WARN(
        get_logger(),
        "Automatic real-motion trigger is enabled. Waiting for a fresh, stable %s TF "
        "for %.2f seconds; execution will occur once only.",
        target_frame_.c_str(), auto_target_stability_seconds_);
    }
  }

private:
  struct Targets
  {
    geometry_msgs::msg::Pose precontact;
    geometry_msgs::msg::Pose contact;
    tf2::Vector3 outward_normal;
    tf2::Vector3 tag_position;
    double current_to_precontact{0.0};
    double current_to_contact{0.0};
    double precontact_to_contact{0.0};
  };

  static double clampScale(double value)
  {
    return std::max(0.01, std::min(value, 1.0));
  }

  static double distance(
    const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b)
  {
    return std::sqrt(
      std::pow(a.x - b.x, 2.0) + std::pow(a.y - b.y, 2.0) +
      std::pow(a.z - b.z, 2.0));
  }

  static geometry_msgs::msg::Point toPoint(const tf2::Vector3 & value)
  {
    geometry_msgs::msg::Point result;
    result.x = value.x();
    result.y = value.y();
    result.z = value.z();
    return result;
  }

  void validateParameters()
  {
    if (target_offset_base_xyz_.size() != 3) {
      throw std::runtime_error("target_offset_base_xyz must contain exactly 3 values");
    }
    if (return_joint_values_.size() != 7) {
      throw std::runtime_error("return_joint_values must contain exactly 7 values");
    }
    if (precontact_distance_ <= contact_standoff_ || contact_standoff_ < 0.0) {
      throw std::runtime_error(
              "precontact_distance must be greater than contact_standoff, and both must be non-negative");
    }
    if (eef_step_ <= 0.0 || minimum_cartesian_fraction_ <= 0.0 ||
      minimum_cartesian_fraction_ > 1.0 || velocity_scale_ <= 0.0 ||
      acceleration_scale_ <= 0.0 || planning_time_seconds_ <= 0.0 ||
      planning_attempts_ <= 0 || tf_timeout_seconds_ <= 0.0 ||
      target_max_age_seconds_ <= 0.0 || maximum_precontact_translation_ <= 0.0 ||
      maximum_contact_translation_ <= 0.0 || driver_speed_ <= 0 ||
      driver_speed_ > 100 || driver_result_timeout_seconds_ <= 0.0 ||
      auto_target_stability_seconds_ < 0.0 ||
      auto_target_position_threshold_ <= 0.0)
    {
      throw std::runtime_error("motion, TF and safety-limit parameters must be positive");
    }
  }

  void autoExecuteTick()
  {
    if (auto_request_sent_ || !auto_execute_client_) {
      return;
    }

    try {
      const auto tag = lookupFreshTag();
      const tf2::Vector3 position(
        tag.transform.translation.x, tag.transform.translation.y,
        tag.transform.translation.z);
      const auto steady_now = std::chrono::steady_clock::now();

      if (!auto_target_seen_ ||
        (position - auto_last_target_position_).length() >
        auto_target_position_threshold_)
      {
        auto_target_seen_ = true;
        auto_last_target_position_ = position;
        auto_target_stable_since_ = steady_now;
        return;
      }

      auto_last_target_position_ = position;
      const double stable_seconds = std::chrono::duration<double>(
        steady_now - auto_target_stable_since_).count();
      if (stable_seconds < auto_target_stability_seconds_ ||
        !auto_execute_client_->service_is_ready())
      {
        return;
      }

      // robot_state_publisher can briefly publish a default/partial Link7
      // transform while the real driver joint state and hand-eye tree are
      // still joining. Validate the complete target without moving first.
      // A transient invalid transform must not consume the one-shot trigger.
      try {
        (void)computeTargets();
      } catch (const std::exception & error) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 3000,
          "Automatic target is not ready yet; continuing to wait: %s",
          error.what());
        auto_target_seen_ = false;
        return;
      }

      auto_request_sent_ = true;
      auto_execute_timer_->cancel();
      RCLCPP_WARN(
        get_logger(),
        "Stable %s target acquired. Automatically calling %s now.",
        target_frame_.c_str(), auto_execute_client_->get_service_name());
      auto_execute_client_->async_send_request(
        std::make_shared<std_srvs::srv::Trigger::Request>(),
        [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
          try {
            const auto response = future.get();
            if (response->success) {
              RCLCPP_INFO(
                get_logger(), "Automatic motion completed: %s",
                response->message.c_str());
            } else {
              RCLCPP_ERROR(
                get_logger(), "Automatic motion failed: %s",
                response->message.c_str());
            }
          } catch (const std::exception & error) {
            RCLCPP_ERROR(
              get_logger(), "Automatic execute service call failed: %s", error.what());
          }
        });
    } catch (const std::exception &) {
      // Camera/tag startup is asynchronous. Keep waiting silently until a
      // fresh connected TF exists; no motion request has been sent yet.
      auto_target_seen_ = false;
    }
  }

  geometry_msgs::msg::TransformStamped lookupFreshTag()
  {
    const auto transform = tf_buffer_.lookupTransform(
      base_frame_, target_frame_, tf2::TimePointZero,
      tf2::durationFromSec(tf_timeout_seconds_));
    const rclcpp::Time stamp(transform.header.stamp, get_clock()->get_clock_type());
    const double age = (now() - stamp).seconds();
    if (age < -0.05 || age > target_max_age_seconds_) {
      throw std::runtime_error(
              "tag TF is stale or clocks are not synchronized; age=" +
              std::to_string(age) + " s");
    }
    return transform;
  }

  Targets computeTargets()
  {
    if (!move_group_) {
      throw std::runtime_error("MoveIt is not initialized");
    }

    const auto tag_msg = lookupFreshTag();
    const auto link_msg = tf_buffer_.lookupTransform(
      base_frame_, end_effector_link_, tf2::TimePointZero,
      tf2::durationFromSec(tf_timeout_seconds_));

    tf2::Transform base_to_tag;
    tf2::Transform base_to_link;
    tf2::fromMsg(tag_msg.transform, base_to_tag);
    tf2::fromMsg(link_msg.transform, base_to_link);

    const tf2::Vector3 tag_position = base_to_tag.getOrigin();
    const tf2::Vector3 link_position = base_to_link.getOrigin();

    // Do not assume whether apriltag_ros publishes +Z into or out of the paper.
    // Select the tag Z direction that points from the tag toward the current
    // Link7.  This always places precontact on the robot side of the tag plane.
    tf2::Vector3 normal = tf2::quatRotate(
      base_to_tag.getRotation(), tf2::Vector3(0.0, 0.0, 1.0));
    normal.normalize();
    const double raw_dot = normal.dot(link_position - tag_position);
    const int selected_sign = raw_dot >= 0.0 ? 1 : -1;
    normal *= static_cast<double>(selected_sign);

    const tf2::Vector3 base_offset(
      target_offset_base_xyz_[0], target_offset_base_xyz_[1],
      target_offset_base_xyz_[2]);
    const tf2::Vector3 contact_position =
      tag_position + base_offset + normal * contact_standoff_;
    const tf2::Vector3 precontact_position =
      tag_position + base_offset + normal * precontact_distance_;

    // Keep the live Link7 orientation for both stages.  The tag in-plane yaw
    // therefore cannot cause joint7 to flip.  Only position and the verified
    // tag normal determine this first test.
    geometry_msgs::msg::Quaternion orientation = link_msg.transform.rotation;

    Targets targets;
    targets.precontact.position = toPoint(precontact_position);
    targets.precontact.orientation = orientation;
    targets.contact.position = toPoint(contact_position);
    targets.contact.orientation = orientation;
    targets.outward_normal = normal;
    targets.tag_position = tag_position;
    targets.current_to_precontact = distance(
      toPoint(link_position), targets.precontact.position);
    targets.current_to_contact = distance(
      toPoint(link_position), targets.contact.position);
    targets.precontact_to_contact = distance(
      targets.precontact.position, targets.contact.position);

    const double first_command_translation = single_stage_to_contact_ ?
      targets.current_to_contact : targets.current_to_precontact;
    if (first_command_translation > maximum_precontact_translation_) {
      throw std::runtime_error(
              std::string(single_stage_to_contact_ ?
              "direct contact translation exceeds safety limit: " :
              "precontact translation exceeds safety limit: ") +
              std::to_string(first_command_translation) + " m");
    }
    if (targets.precontact_to_contact > maximum_contact_translation_) {
      throw std::runtime_error(
              "contact translation exceeds safety limit: " +
              std::to_string(targets.precontact_to_contact) + " m");
    }

    RCLCPP_INFO(
      get_logger(),
      "Tag=[%.4f %.4f %.4f], raw tag-Z dot(tag->Link7)=%.4f, selected sign=%+d, "
      "outward normal=[%.4f %.4f %.4f]",
      tag_position.x(), tag_position.y(), tag_position.z(), raw_dot,
      selected_sign, normal.x(), normal.y(), normal.z());
    RCLCPP_INFO(
      get_logger(),
      "Precontact=[%.4f %.4f %.4f] (travel %.4f m), contact=[%.4f %.4f %.4f] "
      "(direct travel %.4f m, straight approach %.4f m, tag standoff %.4f m)",
      targets.precontact.position.x, targets.precontact.position.y,
      targets.precontact.position.z, targets.current_to_precontact,
      targets.contact.position.x, targets.contact.position.y,
      targets.contact.position.z, targets.current_to_contact,
      targets.precontact_to_contact,
      contact_standoff_);
    return targets;
  }

  bool planPrecontact(
    const geometry_msgs::msg::Pose & pose,
    moveit::planning_interface::MoveGroupInterface::Plan & plan)
  {
    move_group_->setStartStateToCurrentState();
    move_group_->setPlanningPipelineId("pilz_industrial_motion_planner");
    move_group_->setPlannerId("PTP");
    move_group_->setPoseTarget(pose, end_effector_link_);
    const bool success =
      move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS;
    move_group_->clearPoseTargets();
    if (!success) {
      RCLCPP_ERROR(
        get_logger(),
        "Pilz PTP could not plan the current-orientation precontact pose. "
        "No fallback to position-only IK or RRT will be executed");
    }
    return success;
  }

  bool cartesianTo(const geometry_msgs::msg::Pose & pose, bool execute_motion)
  {
    move_group_->setStartStateToCurrentState();
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double fraction = move_group_->computeCartesianPath(
      std::vector<geometry_msgs::msg::Pose>{pose}, eef_step_, jump_threshold_,
      trajectory, true);
    RCLCPP_INFO(
      get_logger(), "Cartesian coverage %.3f (required %.3f)", fraction,
      minimum_cartesian_fraction_);
    if (fraction + 1e-6 < minimum_cartesian_fraction_) {
      RCLCPP_ERROR(
        get_logger(),
        "The straight segment is not fully reachable/collision-free; partial trajectory will not execute");
      return false;
    }
    if (!execute_motion) {
      return true;
    }
    return move_group_->execute(trajectory) == moveit::core::MoveItErrorCode::SUCCESS;
  }

  bool returnToConfiguredJointPose()
  {
    RCLCPP_INFO(get_logger(), "Planning return to the configured seven-joint pose");
    move_group_->setStartStateToCurrentState();
    move_group_->setPlanningPipelineId("pilz_industrial_motion_planner");
    move_group_->setPlannerId("PTP");
    if (!move_group_->setJointValueTarget(return_joint_values_)) {
      RCLCPP_ERROR(get_logger(), "Configured return joint target is invalid");
      return false;
    }
    moveit::planning_interface::MoveGroupInterface::Plan return_plan;
    if (move_group_->plan(return_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Failed to plan the configured joint return");
      return false;
    }
    if (move_group_->execute(return_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Failed to execute the configured joint return");
      return false;
    }
    RCLCPP_INFO(get_logger(), "Returned to the configured seven-joint pose");
    return true;
  }

  bool motionUnlocked(std::string & reason) const
  {
    if (!allow_motion_) {
      reason = "Motion is locked: set allow_motion:=true at launch";
      return false;
    }
    if (dry_run_) {
      reason = "dry_run is true; only plans will be generated";
      return false;
    }
    return true;
  }

  geometry_msgs::msg::Pose latestDriverPose()
  {
    std::lock_guard<std::mutex> lock(arm_pose_mutex_);
    if (!arm_pose_received_) {
      throw std::runtime_error(
              "no /rm_driver/udp_arm_position received; rm_driver is not ready");
    }
    return latest_arm_pose_;
  }

  geometry_msgs::msg::Pose driverTargetForLinkTarget(
    const geometry_msgs::msg::Pose & desired_link_pose)
  {
    const auto current_driver_pose = latestDriverPose();
    const auto current_link = tf_buffer_.lookupTransform(
      base_frame_, end_effector_link_, tf2::TimePointZero,
      tf2::durationFromSec(tf_timeout_seconds_));

    const double dx = desired_link_pose.position.x - current_link.transform.translation.x;
    const double dy = desired_link_pose.position.y - current_link.transform.translation.y;
    const double dz = desired_link_pose.position.z - current_link.transform.translation.z;

    geometry_msgs::msg::Pose driver_target = current_driver_pose;
    driver_target.position.x += dx;
    driver_target.position.y += dy;
    driver_target.position.z += dz;
    RCLCPP_INFO(
      get_logger(),
      "Link7 translation command=[%.4f %.4f %.4f] m; controller TCP current="
      "[%.4f %.4f %.4f], target=[%.4f %.4f %.4f]",
      dx, dy, dz, current_driver_pose.position.x, current_driver_pose.position.y,
      current_driver_pose.position.z, driver_target.position.x,
      driver_target.position.y, driver_target.position.z);
    return driver_target;
  }

  bool sendMoveJP(geometry_msgs::msg::Pose pose)
  {
    // Use the controller's own live quaternion.  RM MoveJ_P then performs the
    // redundant-arm IK using the physical controller rather than MoveIt KDL.
    pose = driverTargetForLinkTarget(pose);
    {
      std::lock_guard<std::mutex> lock(driver_result_mutex_);
      movejp_result_received_ = false;
    }
    rm_ros_interfaces::msg::Movejp command;
    command.pose = pose;
    command.speed = static_cast<uint8_t>(driver_speed_);
    command.trajectory_connect = 0;
    command.block = true;
    movejp_publisher_->publish(command);
    RCLCPP_INFO(get_logger(), "Published blocking RM MoveJ_P command");

    std::unique_lock<std::mutex> lock(driver_result_mutex_);
    if (!driver_result_cv_.wait_for(
        lock, std::chrono::duration<double>(driver_result_timeout_seconds_),
        [this]() {return movejp_result_received_;}))
    {
      RCLCPP_ERROR(get_logger(), "Timed out waiting for /rm_driver/movej_p_result");
      return false;
    }
    return movejp_result_;
  }

  bool sendMoveL(geometry_msgs::msg::Pose pose)
  {
    pose = driverTargetForLinkTarget(pose);
    {
      std::lock_guard<std::mutex> lock(driver_result_mutex_);
      movel_result_received_ = false;
    }
    rm_ros_interfaces::msg::Movel command;
    command.pose = pose;
    command.speed = static_cast<uint8_t>(driver_speed_);
    command.trajectory_connect = 0;
    command.block = true;
    movel_publisher_->publish(command);
    RCLCPP_INFO(get_logger(), "Published blocking RM MoveL command");

    std::unique_lock<std::mutex> lock(driver_result_mutex_);
    if (!driver_result_cv_.wait_for(
        lock, std::chrono::duration<double>(driver_result_timeout_seconds_),
        [this]() {return movel_result_received_;}))
    {
      RCLCPP_ERROR(get_logger(), "Timed out waiting for /rm_driver/movel_result");
      return false;
    }
    return movel_result_;
  }

  void previewCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    if (busy_.exchange(true)) {
      response->message = "Another motion request is running";
      return;
    }
    try {
      const auto targets = computeTargets();
      if (use_rm_driver_backend_) {
        (void)latestDriverPose();
        response->success = true;
        response->message =
          "Target geometry is valid. RM driver backend does not execute during preview";
        busy_ = false;
        return;
      }
      moveit::planning_interface::MoveGroupInterface::Plan precontact_plan;
      response->success = planPrecontact(targets.precontact, precontact_plan);
      response->message = response->success ?
        "Precontact PTP plan succeeded. No robot motion was executed" :
        "Precontact PTP planning failed";
    } catch (const std::exception & error) {
      response->success = false;
      response->message = error.what();
      RCLCPP_ERROR(get_logger(), "Preview failed: %s", error.what());
    }
    busy_ = false;
  }

  void executeCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    if (busy_.exchange(true)) {
      response->message = "Another motion request is running";
      return;
    }
    try {
      const auto targets = computeTargets();
      if (use_rm_driver_backend_) {
        (void)latestDriverPose();
        std::string lock_reason;
        if (!motionUnlocked(lock_reason)) {
          response->success = true;
          response->message = "Target geometry is valid. " + lock_reason +
            "; robot did not move";
          busy_ = false;
          return;
        }
        const auto & first_target = single_stage_to_contact_ ?
          targets.contact : targets.precontact;
        if (!sendMoveJP(first_target)) {
          throw std::runtime_error(
                  single_stage_to_contact_ ?
                  "RM controller rejected/failed the direct MoveJ_P contact command" :
                  "RM controller rejected/failed the MoveJ_P precontact command");
        }
        if (single_stage_to_contact_) {
          RCLCPP_INFO(get_logger(), "RM controller reached contact in one MoveJ_P stage");
          if (auto_return_after_execute_ && !returnToConfiguredJointPose()) {
            throw std::runtime_error("button motion succeeded but configured joint return failed");
          }
          response->success = true;
          response->message = auto_return_after_execute_ ?
            "RM MoveJ_P reached contact and robot returned to configured joints" :
            "RM MoveJ_P reached the final contact target directly";
          busy_ = false;
          return;
        }
        RCLCPP_INFO(get_logger(), "RM controller reached precontact");
        if (execute_contact_) {
          // Keep using the target pair captured before motion.  With an
          // eye-in-hand camera the tag may leave the image at precontact; a
          // second TF lookup would then abort a valid, already planned move.
          if (!sendMoveL(targets.contact)) {
            throw std::runtime_error("RM controller rejected/failed the MoveL contact command");
          }
          RCLCPP_INFO(get_logger(), "RM controller reached configured tag standoff");

          RCLCPP_INFO(
              get_logger(),
              "Button contact completed; dwelling for 1 second");
            rclcpp::sleep_for(1s);

          if (auto_retract_ && !sendMoveL(targets.precontact)) {
            throw std::runtime_error("RM controller MoveL retract failed");
          }
        }

        if (auto_return_after_execute_) {
          RCLCPP_INFO(get_logger(), "Button contact completed; dwelling for 1 second");
          rclcpp::sleep_for(1s);
        }

        if (auto_return_after_execute_ && !returnToConfiguredJointPose()) {
          throw std::runtime_error("button motion succeeded but configured joint return failed");
        }
        response->success = true;
        response->message = auto_return_after_execute_ ?
          "Button motion completed and robot returned to configured joints" : execute_contact_ ?
          "RM MoveJ_P reached precontact and RM MoveL completed the straight approach" :
          "RM MoveJ_P reached precontact; contact approach is disabled";
        busy_ = false;
        return;
      }
      moveit::planning_interface::MoveGroupInterface::Plan precontact_plan;
      const auto & first_target = single_stage_to_contact_ ?
        targets.contact : targets.precontact;
      if (!planPrecontact(first_target, precontact_plan)) {
        throw std::runtime_error(
                single_stage_to_contact_ ?
                "direct contact PTP planning failed" : "precontact PTP planning failed");
      }

      std::string lock_reason;
      if (!motionUnlocked(lock_reason)) {
        response->success = true;
        response->message = "Plan succeeded. " + lock_reason + "; robot did not move";
        busy_ = false;
        return;
      }

      if (move_group_->execute(precontact_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
        throw std::runtime_error("execution to precontact failed");
      }
      if (single_stage_to_contact_) {
        RCLCPP_INFO(get_logger(), "Reached contact in one PTP stage");
        if (auto_return_after_execute_ && !returnToConfiguredJointPose()) {
          throw std::runtime_error("button motion succeeded but configured joint return failed");
        }
        response->success = true;
        response->message = auto_return_after_execute_ ?
          "Reached contact and returned to configured joints" :
          "Reached the final contact target directly with PTP";
        busy_ = false;
        return;
      }
      RCLCPP_INFO(get_logger(), "Reached precontact pose");

      if (execute_contact_) {
        // Execute the contact waypoint captured before motion.  Do not require
        // the eye-in-hand camera to keep seeing the tag at precontact.
        if (!cartesianTo(targets.contact, true)) {
          throw std::runtime_error("Cartesian approach to contact target failed");
        }
        RCLCPP_INFO(get_logger(), "Reached configured tag standoff");
        if (auto_retract_ && !cartesianTo(targets.precontact, true)) {
          throw std::runtime_error("automatic Cartesian retract failed");
        }
      }

      if (auto_return_after_execute_ && !returnToConfiguredJointPose()) {
        throw std::runtime_error("button motion succeeded but configured joint return failed");
      }

      response->success = true;
      response->message = auto_return_after_execute_ ?
        "Button motion completed and robot returned to configured joints" : execute_contact_ ?
        "Reached precontact and completed the straight tag-normal approach" :
        "Reached precontact; contact approach is disabled";
    } catch (const std::exception & error) {
      response->success = false;
      response->message = error.what();
      RCLCPP_ERROR(get_logger(), "Execution failed: %s", error.what());
    }
    busy_ = false;
  }

  void retractCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    if (busy_.exchange(true)) {
      response->message = "Another motion request is running";
      return;
    }
    try {
      std::string reason;
      if (!motionUnlocked(reason)) {
        throw std::runtime_error(reason);
      }
      const auto targets = computeTargets();
      response->success = use_rm_driver_backend_ ?
        sendMoveL(targets.precontact) : cartesianTo(targets.precontact, true);
      response->message = response->success ?
        "Retracted along the tag normal" : "Cartesian retract failed";
    } catch (const std::exception & error) {
      response->success = false;
      response->message = error.what();
      RCLCPP_ERROR(get_logger(), "Retract failed: %s", error.what());
    }
    busy_ = false;
  }

  void returnCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    if (busy_.exchange(true)) {
      response->message = "Another motion request is running";
      return;
    }
    try {
      std::string reason;
      if (!motionUnlocked(reason)) {
        throw std::runtime_error(reason);
      }
      move_group_->setStartStateToCurrentState();
      move_group_->setPlanningPipelineId("pilz_industrial_motion_planner");
      move_group_->setPlannerId("PTP");
      if (!move_group_->setNamedTarget(return_named_target_)) {
        throw std::runtime_error("unknown named target: " + return_named_target_);
      }
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      if (move_group_->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
        throw std::runtime_error("return-home PTP planning failed");
      }
      response->success =
        move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
      response->message = response->success ?
        "Returned to named target " + return_named_target_ :
        "Return-home execution failed";
    } catch (const std::exception & error) {
      response->success = false;
      response->message = error.what();
      RCLCPP_ERROR(get_logger(), "Return-home failed: %s", error.what());
    }
    busy_ = false;
  }

  std::string planning_group_;
  std::string base_frame_;
  std::string end_effector_link_;
  std::string target_frame_;
  std::string return_named_target_;
  std::vector<double> return_joint_values_;
  bool auto_return_after_execute_{false};
  std::vector<double> target_offset_base_xyz_;
  double precontact_distance_{0.10};
  double contact_standoff_{0.05};
  bool execute_contact_{true};
  bool auto_retract_{false};
  bool single_stage_to_contact_{false};
  double eef_step_{0.005};
  double jump_threshold_{0.0};
  double minimum_cartesian_fraction_{0.98};
  double position_tolerance_{0.01};
  double orientation_tolerance_{0.10};
  double velocity_scale_{0.05};
  double acceleration_scale_{0.05};
  double planning_time_seconds_{10.0};
  int planning_attempts_{5};
  double tf_timeout_seconds_{1.0};
  double target_max_age_seconds_{0.75};
  double maximum_precontact_translation_{0.45};
  double maximum_contact_translation_{0.15};
  bool use_rm_driver_backend_{true};
  int driver_speed_{10};
  double driver_result_timeout_seconds_{90.0};
  bool allow_motion_{false};
  bool dry_run_{true};
  bool auto_execute_{false};
  double auto_target_stability_seconds_{1.5};
  double auto_target_position_threshold_{0.01};
  bool auto_target_seen_{false};
  bool auto_request_sent_{false};
  tf2::Vector3 auto_last_target_position_{0.0, 0.0, 0.0};
  std::chrono::steady_clock::time_point auto_target_stable_since_;
  std::atomic_bool busy_{false};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  rclcpp::Publisher<rm_ros_interfaces::msg::Movejp>::SharedPtr movejp_publisher_;
  rclcpp::Publisher<rm_ros_interfaces::msg::Movel>::SharedPtr movel_publisher_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr movejp_result_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr movel_result_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr arm_pose_subscription_;
  std::mutex driver_result_mutex_;
  std::condition_variable driver_result_cv_;
  bool movejp_result_received_{false};
  bool movejp_result_{false};
  bool movel_result_received_{false};
  bool movel_result_{false};
  std::mutex arm_pose_mutex_;
  geometry_msgs::msg::Pose latest_arm_pose_;
  bool arm_pose_received_{false};
  rclcpp::CallbackGroup::SharedPtr service_callback_group_;
  rclcpp::CallbackGroup::SharedPtr auto_callback_group_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr auto_execute_client_;
  rclcpp::TimerBase::SharedPtr auto_execute_timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr execute_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr preview_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr retract_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr return_service_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FixedGraspNode>();
  node->initializeMoveIt();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}

// Copyright 2026
// PoseStamped topic bridge for OMY F3M MoveJ-style MoveIt execution.

#include <memory>
#include <mutex>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <rclcpp/rclcpp.hpp>

class OMYF3MMoveJPoseTopic : public rclcpp::Node
{
public:
  OMYF3MMoveJPoseTopic()
  : Node("omy_f3m_movej_pose_topic", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
  {
    this->get_parameter_or<std::string>("target_pose_topic", target_pose_topic_, "/omy/target_pose");
    this->get_parameter_or<std::string>("planning_group", planning_group_, "arm");
    this->get_parameter_or<double>("position_tolerance", position_tolerance_, 0.01);
    this->get_parameter_or<double>("orientation_tolerance", orientation_tolerance_, 0.01);
    this->get_parameter_or<double>("planning_time", planning_time_, 5.0);
    this->get_parameter_or<double>("max_velocity_scaling_factor", max_velocity_scaling_factor_, 0.1);
    this->get_parameter_or<double>("max_acceleration_scaling_factor", max_acceleration_scaling_factor_, 0.1);
  }

  void initialize()
  {
    move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), planning_group_);
    move_group_->setGoalPositionTolerance(position_tolerance_);
    move_group_->setGoalOrientationTolerance(orientation_tolerance_);
    move_group_->setPlanningTime(planning_time_);
    move_group_->setMaxVelocityScalingFactor(max_velocity_scaling_factor_);
    move_group_->setMaxAccelerationScalingFactor(max_acceleration_scaling_factor_);

    target_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      target_pose_topic_, 10,
      std::bind(&OMYF3MMoveJPoseTopic::target_pose_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Listening on %s", target_pose_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "MoveIt planning group: %s", planning_group_.c_str());
    RCLCPP_INFO(this->get_logger(), "Planning frame: %s", move_group_->getPlanningFrame().c_str());
    RCLCPP_INFO(this->get_logger(), "End effector link: %s", move_group_->getEndEffectorLink().c_str());
  }

private:
  void target_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(move_mutex_);

    geometry_msgs::msg::PoseStamped target = *msg;
    if (target.header.frame_id.empty()) {
      target.header.frame_id = move_group_->getPlanningFrame();
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Received target pose frame=%s position=[%.4f, %.4f, %.4f] orientation=[%.4f, %.4f, %.4f, %.4f]",
      target.header.frame_id.c_str(),
      target.pose.position.x, target.pose.position.y, target.pose.position.z,
      target.pose.orientation.x, target.pose.orientation.y, target.pose.orientation.z, target.pose.orientation.w);

    move_group_->setPoseTarget(target);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const bool planned = static_cast<bool>(move_group_->plan(plan));
    if (!planned) {
      RCLCPP_ERROR(this->get_logger(), "MOVEJ_POSE_PLAN_FAILED");
      move_group_->clearPoseTargets();
      return;
    }

    RCLCPP_INFO(this->get_logger(), "MOVEJ_POSE_PLAN_SUCCEEDED. Executing...");
    const bool executed = static_cast<bool>(move_group_->execute(plan));
    if (!executed) {
      RCLCPP_ERROR(this->get_logger(), "MOVEJ_POSE_EXECUTE_FAILED");
      move_group_->clearPoseTargets();
      return;
    }

    RCLCPP_INFO(this->get_logger(), "MOVEJ_POSE_EXECUTE_SUCCEEDED");
    move_group_->clearPoseTargets();
  }

  std::string target_pose_topic_;
  std::string planning_group_;
  double position_tolerance_;
  double orientation_tolerance_;
  double planning_time_;
  double max_velocity_scaling_factor_;
  double max_acceleration_scaling_factor_;

  std::mutex move_mutex_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OMYF3MMoveJPoseTopic>();
  node->initialize();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

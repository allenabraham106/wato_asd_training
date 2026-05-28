#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <utility>

#include "control_node.hpp"

namespace {
constexpr double kPi = 3.14159265358979323846;
}

ControlNode::ControlNode()
: Node("control"), control_(robot::ControlCore(this->get_logger())),
  have_path_(false), have_odom_(false) {
  lookahead_distance_ = 0.8;
  goal_tolerance_ = 0.5;
  linear_speed_ = 0.8;
  max_angular_speed_ = 1.5;

  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/path", 10, std::bind(&ControlNode::pathCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&ControlNode::odomCallback, this, std::placeholders::_1));
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  control_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100), std::bind(&ControlNode::controlLoop, this));
}

void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
  current_path_ = *msg;
  have_path_ = true;
}

void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  current_odom_ = *msg;
  have_odom_ = true;
}

void ControlNode::controlLoop() {
  if (!have_path_ || !have_odom_) return;

  geometry_msgs::msg::Twist cmd;
  if (current_path_.poses.empty() || distanceToFinalGoal() <= goal_tolerance_) {
    cmd_vel_pub_->publish(cmd);
    return;
  }

  const auto lookahead = findLookaheadPoint();
  if (!lookahead) {
    cmd_vel_pub_->publish(cmd);
    return;
  }

  cmd = computeVelocity(*lookahead);
  cmd_vel_pub_->publish(cmd);
}

std::optional<geometry_msgs::msg::PoseStamped> ControlNode::findLookaheadPoint() const {
  if (current_path_.poses.empty()) return std::nullopt;

  const auto start_index = closestPathIndex();
  double distance_along_path = 0.0;

  for (std::size_t i = start_index + 1; i < current_path_.poses.size(); ++i) {
    const auto & previous = current_path_.poses[i - 1].pose.position;
    const auto & current = current_path_.poses[i].pose.position;
    distance_along_path += std::hypot(current.x - previous.x, current.y - previous.y);
    if (distance_along_path >= lookahead_distance_) {
      return current_path_.poses[i];
    }
  }
  return current_path_.poses.back();
}

geometry_msgs::msg::Twist ControlNode::computeVelocity(const geometry_msgs::msg::PoseStamped & target) const {
  geometry_msgs::msg::Twist cmd;
  const auto & robot_pose = current_odom_.pose.pose;
  const double yaw = yawFromQuaternion(robot_pose.orientation);
  const double dx = target.pose.position.x - robot_pose.position.x;
  const double dy = target.pose.position.y - robot_pose.position.y;
  const double target_heading = std::atan2(dy, dx);
  const double alpha = normalizeAngle(target_heading - yaw);
  const double distance = std::hypot(dx, dy);
  const double effective_lookahead = std::max(distance, 0.001);
  const double curvature = 2.0 * std::sin(alpha) / effective_lookahead;

  if (std::abs(alpha) > kPi / 2.0) {
    cmd.linear.x = 0.0;
    cmd.angular.z = std::clamp(2.0 * alpha, -max_angular_speed_, max_angular_speed_);
    return cmd;
  }

  cmd.linear.x = linear_speed_ * std::max(0.25, std::cos(alpha));
  cmd.angular.z = std::clamp(cmd.linear.x * curvature, -max_angular_speed_, max_angular_speed_);
  return cmd;
}

std::size_t ControlNode::closestPathIndex() const {
  const auto & robot_position = current_odom_.pose.pose.position;
  std::size_t closest_index = 0;
  double closest_distance = std::numeric_limits<double>::max();

  for (std::size_t i = 0; i < current_path_.poses.size(); ++i) {
    const auto & path_position = current_path_.poses[i].pose.position;
    const double distance = std::hypot(path_position.x - robot_position.x, path_position.y - robot_position.y);
    if (distance < closest_distance) {
      closest_distance = distance;
      closest_index = i;
    }
  }
  return closest_index;
}

double ControlNode::distanceToFinalGoal() const {
  if (current_path_.poses.empty()) return 0.0;
  const auto & robot_position = current_odom_.pose.pose.position;
  const auto & goal_position = current_path_.poses.back().pose.position;
  return std::hypot(goal_position.x - robot_position.x, goal_position.y - robot_position.y);
}

double ControlNode::yawFromQuaternion(const geometry_msgs::msg::Quaternion & quat) const {
  const double siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y);
  const double cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

double ControlNode::normalizeAngle(double angle) const {
  while (angle > kPi) angle -= 2.0 * kPi;
  while (angle < -kPi) angle += 2.0 * kPi;
  return angle;
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
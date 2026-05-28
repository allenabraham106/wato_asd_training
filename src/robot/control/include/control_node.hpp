#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "control_core.hpp"
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <optional>
#include <cstddef>

class ControlNode : public rclcpp::Node {
  public:
    ControlNode();

  private:
    robot::ControlCore control_;
    
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void controlLoop();
    std::optional<geometry_msgs::msg::PoseStamped> findLookaheadPoint() const;
    geometry_msgs::msg::Twist computeVelocity(const geometry_msgs::msg::PoseStamped &target) const;
    std::size_t closestPathIndex() const;
    double distanceToFinalGoal() const;
    double yawFromQuaternion(const geometry_msgs::msg::Quaternion &quat) const;
    double normalizeAngle(double angle) const;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    nav_msgs::msg::Path current_path_;
    nav_msgs::msg::Odometry current_odom_;
    bool have_path_ = false;
    bool have_odom_ = false;

    double lookahead_distance_ = 0.8;
    double goal_tolerance_ = 0.5;
    double linear_speed_ = 0.45;
    double max_angular_speed_ = 1.5;
};

#endif
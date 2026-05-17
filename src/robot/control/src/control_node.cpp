#include "control_node.hpp"

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore(this->get_logger())) {
  lookahead_distance_ = 2.0;  // Lookahead distance
        goal_tolerance_ = 1.0;     // Distance to consider the goal reached
        linear_speed_ = 1.0;       // Constant forward speed
 
        // Subscribers and Publishers
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/path", 10, [this](const nav_msgs::msg::Path::SharedPtr msg) { current_path_ = msg; });
 
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom/filtered", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) { robot_odom_ = msg; });
 
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
 
        // Timer
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), [this]() { controlLoop(); });
}

void ControlNode::controlLoop() {
        // Skip control if no path or odometry data is available

    if (!current_path_ || !robot_odom_) {
        RCLCPP_WARN(this->get_logger(), "No path!");
        return;
    }
    if (current_path_->poses.empty()) {
        RCLCPP_WARN(this->get_logger(), "Empty path!");
        return;
    }
  if (!current_path_ || !robot_odom_) {
    return;
  }
 
        // Find the lookahead point
  auto lookahead_point = findLookaheadPoint();
  if (!lookahead_point) {
    return;  // No valid lookahead point found
  }

  auto last_pose = current_path_->poses.back();
  geometry_msgs::msg::Point robot_pos = robot_odom_->pose.pose.position;
  double dist_to_end = computeDistance(robot_pos, last_pose.pose.position);
  if (dist_to_end < 0.5) {
    geometry_msgs::msg::Twist stop;
    cmd_vel_pub_->publish(stop);
    return;
  }
 
        // Compute velocity command
  auto cmd_vel = computeVelocity(*lookahead_point);
 
        // Publish the velocity command
  cmd_vel_pub_->publish(cmd_vel);

}


std::optional<geometry_msgs::msg::PoseStamped> ControlNode::findLookaheadPoint() {
  // TODO: Implement logic to find the lookahead point on the path
  if(!current_path_ || current_path_->poses.empty()){
    return std::nullopt;
  }

  geometry_msgs::msg::Point robot_pos = robot_odom_->pose.pose.position;
  for(const auto& pose : current_path_ -> poses){
    double dist = computeDistance(robot_pos, pose.pose.position);
    if(dist >= lookahead_distance_){
      return pose;
    }
  }
  return current_path_->poses.back();  // Replace with a valid point when implemented
}

geometry_msgs::msg::Twist ControlNode::computeVelocity(const geometry_msgs::msg::PoseStamped &target) {
 // TODO: Implement logic to compute velocity commands
  geometry_msgs::msg::Twist cmd_vel;

  geometry_msgs::msg::Point robot_pos = robot_odom_->pose.pose.position;
  double robot_yaw = extractYaw(robot_odom_ -> pose.pose.orientation); 

  double dx = target.pose.position.x - robot_pos.x;
  double dy = target.pose.position.y - robot_pos.y; 
  double angle_to_target = std::atan2(dy, dx);

  double angle_error = angle_to_target - robot_yaw;

  while(angle_error > M_PI){
    angle_error -= 2 * M_PI;
  }

  while(angle_error < -M_PI){
    angle_error += 2 * M_PI;
  }

  double dist_to_goal = computeDistance(robot_pos, current_path_ -> poses.back().pose.position);

  if(dist_to_goal < goal_tolerance_){
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
    return cmd_vel;
  }

  cmd_vel.linear.x = linear_speed_;
  cmd_vel.angular.z = std::max(-1.0, std::min(1.0, 2.0 * angle_error));

  return cmd_vel;
}

double ControlNode::computeDistance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b) {
  // TODO: Implement distance calculation between two points
  return std::sqrt(std::pow(a.x-b.x, 2) + std::pow(a.y-b.y,2));
}

double ControlNode::extractYaw(const geometry_msgs::msg::Quaternion &quat) {
  // TODO: Implement quaternion to yaw conversion
  double siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y);
  double cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}

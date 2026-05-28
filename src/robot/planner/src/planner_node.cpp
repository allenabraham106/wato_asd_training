#include "planner_node.hpp"
#include <cmath>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>

PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())) {
  state_ = State::WAITING_FOR_GOAL;
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
        goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));
 
        // Publisher
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
 
        // Timer
        timer_ = this->create_wall_timer(
            std::chrono::seconds(3), std::bind(&PlannerNode::timerCallback, this));
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
  current_map_ = *msg;
  //if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
   // planPath();
  //}
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  goal_ = *msg;
  goal_received_ = true;
  state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
  planPath();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_pose_ = msg->pose.pose;
}
 
void PlannerNode::timerCallback() {
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
      if (goalReached()) {
          RCLCPP_INFO(this->get_logger(), "Goal reached!");
          state_ = State::WAITING_FOR_GOAL;
      } else {
          RCLCPP_INFO(this->get_logger(), "Replanning due to timeout or progress...");
          planPath();
      }
  }
}
 
bool PlannerNode::goalReached() {
  double dx = goal_.point.x - robot_pose_.position.x;
  double dy = goal_.point.y - robot_pose_.position.y;
  return std::sqrt(dx * dx + dy * dy) < 1.0; // Threshold for reaching the goal
}
 
void PlannerNode::planPath() {
  if (!goal_received_ || current_map_.data.empty()) {
    RCLCPP_WARN(this->get_logger(), "Cannot plan path: Missing map or goal!");
    return;
  }
  
  // A* Implementation (pseudo-code)
  nav_msgs::msg::Path path;
  path.header.stamp = this->get_clock()->now();
  path.header.frame_id = "sim_world";
 
  // Compute path using A* on current_map_
  // Fill path.poses with the resulting waypoints.
  CellIndex start;
  start.x = (robot_pose_.position.x / 0.1) + 150;
  start.y = (robot_pose_.position.y / 0.1) + 150;

  CellIndex goal; 
  goal.x = (goal_.point.x / 0.1) + 150;
  goal.y = (goal_.point.y / 0.1) + 150;

  //Sorted by f score
  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_list;
  // Already Evaluated Scores
  std::unordered_set<CellIndex, CellIndexHash> closed_set; 
  // cost to reach each cell
  std::unordered_map<CellIndex, double, CellIndexHash> g_score;
  // reconstruct the path to the end
  std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;


  // start
  g_score[start] = 0.0;
  open_list.push(AStarNode(start, 0.0));

  while(!open_list.empty()){
    AStarNode current = open_list.top();
    open_list.pop();

    if(current.index == goal){
      std::vector<CellIndex> path_indices;
      CellIndex current_index = goal;

      while(current_index != start){
        path_indices.push_back(current_index);
        current_index = came_from[current_index];
      }

      path_indices.push_back(start);

      std::reverse(path_indices.begin(), path_indices.end());

      for(auto& idx : path_indices){
        geometry_msgs::msg::PoseStamped pose; 
        pose.header.frame_id = "sim_world";
        pose.header.stamp = this -> get_clock() -> now();
        pose.pose.position.x = (idx.x - 150) * 0.1;
        pose.pose.position.y = (idx.y - 150) * 0.1;
        path.poses.push_back(pose);
      }
      break;
    }

    if(closed_set.count(current.index)){
      continue;
    }

    closed_set.insert(current.index);

    std::vector<CellIndex> neighbors = {
      {current.index.x + 1, current.index.y},
      {current.index.x - 1, current.index.y},
      {current.index.x, current.index.y + 1},
      {current.index.x, current.index.y -1}
    };

    for(auto& neighbor : neighbors){
      if(neighbor.x < 0 || neighbor.y < 0 || neighbor.x >= 300 || neighbor.y >= 300){
        continue;
      }
      int cell_cost = current_map_.data[neighbor.y * 300 + neighbor.x];
      if((cell_cost > 50) || closed_set.count(neighbor)){
        continue;
      }

      double new_g = g_score[current.index] + 1.0;

      if(!g_score.count(neighbor) || new_g < g_score[neighbor]){
        g_score[neighbor] = new_g;
        double h = std::sqrt(std::pow(neighbor.x - goal.x, 2) + std::pow(neighbor.y - goal.y, 2));
        double f = new_g + h;
        came_from[neighbor] = current.index;
        open_list.push(AStarNode(neighbor, f));
      }
    }
  }
  RCLCPP_INFO(this->get_logger(), "Path has %d poses", (int)path.poses.size());
  RCLCPP_INFO(this->get_logger(), "Map size: %d, Start: %d,%d Goal: %d,%d", 
    (int)current_map_.data.size(), start.x, start.y, goal.x, goal.y);
  if (!path.poses.empty()) {
    path_pub_->publish(path);
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}

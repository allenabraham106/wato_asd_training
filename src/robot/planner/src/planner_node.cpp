#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>
#include <unordered_map>
#include <unordered_set>

#include "planner_node.hpp"

PlannerNode::PlannerNode()
: Node("planner"),
  planner_(robot::PlannerCore(this->get_logger())),
  state_(State::WAITING_FOR_GOAL),
  have_map_(false),
  have_odom_(false),
  have_goal_(false) {
  goal_tolerance_ = this->declare_parameter<double>("goal_tolerance", 0.35);
  replan_period_s_ = this->declare_parameter<double>("replan_period_s", 1.0);
  cost_weight_ = this->declare_parameter<double>("cost_weight", 0.08);
  occupied_threshold_ = this->declare_parameter<int>("occupied_threshold", 65);

  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

  const auto period = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::duration<double>(replan_period_s_));
  timer_ = this->create_wall_timer(period, std::bind(&PlannerNode::timerCallback, this));
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  current_map_ = *msg;
  have_map_ = true;
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    planPath();
  }
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  goal_ = *msg;
  have_goal_ = true;
  state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
  planPath();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_pose_ = msg->pose.pose;
  have_odom_ = true;
}

void PlannerNode::timerCallback() {
  if (state_ != State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    return;
  }

  if (goalReached()) {
    RCLCPP_INFO(this->get_logger(), "Goal reached");
    state_ = State::WAITING_FOR_GOAL;
    publishEmptyPath();
    return;
  }

  planPath();
}

void PlannerNode::planPath() {
  if (!have_map_ || !have_odom_ || !have_goal_ || current_map_.data.empty()) {
    return;
  }
  if (current_map_.data.size() <
      static_cast<std::size_t>(current_map_.info.width * current_map_.info.height)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Map message has invalid dimensions");
    publishEmptyPath();
    return;
  }

  CellIndex start;
  CellIndex goal;
  if (!worldToGrid(robot_pose_.position.x, robot_pose_.position.y, start) ||
      !worldToGrid(goal_.point.x, goal_.point.y, goal)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Start or goal is outside the map");
    publishEmptyPath();
    return;
  }

  if (!isTraversable(start) || !isTraversable(goal)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Start or goal is occupied");
    publishEmptyPath();
    return;
  }

  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_set;
  std::unordered_map<int, CellIndex> came_from;
  std::unordered_map<int, double> g_score;
  std::unordered_set<int> closed;

  const int start_key = cellKey(start);
  g_score[start_key] = 0.0;
  open_set.push({start, heuristic(start, goal)});

  bool found = false;
  CellIndex current = start;

  while (!open_set.empty()) {
    current = open_set.top().index;
    open_set.pop();

    const int current_key = cellKey(current);
    if (closed.count(current_key) > 0){
      continue;
    }

    if (current == goal) {
      found = true;
      break;
    }

    closed.insert(current_key);

    for (const auto & neighbor : getNeighbors(current)){
      const int neighbor_key = cellKey(neighbor);
      if (closed.count(neighbor_key) > 0 || !isTraversable(neighbor) || !canMoveBetween(current, neighbor)) {
        continue;
      }

      const double movement_cost = std::hypot(neighbor.x - current.x, neighbor.y - current.y);
      const int cell_cost = std::max<int>(0, current_map_.data[neighbor.y * current_map_.info.width + neighbor.x]);
      const double tentative_g = g_score[current_key] + movement_cost + cost_weight_ * cell_cost;

      const auto existing = g_score.find(neighbor_key);
      if (existing == g_score.end() || tentative_g < existing->second) {
        came_from[neighbor_key] = current;
        g_score[neighbor_key] = tentative_g;
        open_set.push({neighbor, tentative_g + heuristic(neighbor, goal)});
      }
    }
  }

  if (!found) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "No path found to goal");
    publishEmptyPath();
    return;
  }

  nav_msgs::msg::Path path;
  path.header.stamp = this->get_clock()->now();
  path.header.frame_id = current_map_.header.frame_id.empty() ? "sim_world" : current_map_.header.frame_id;
  for (const auto & cell : reconstructPath(came_from, current)) {
    path.poses.push_back(gridToPose(cell));
  }

  path_pub_->publish(path);
}

void PlannerNode::publishEmptyPath() {
  nav_msgs::msg::Path empty_path;
  empty_path.header.stamp = this->get_clock()->now();
  empty_path.header.frame_id = current_map_.header.frame_id.empty() ? "sim_world" : current_map_.header.frame_id;
  path_pub_->publish(empty_path);
}

bool PlannerNode::goalReached() const {
  if (!have_goal_ || !have_odom_) {
    return false;
  }
  return std::hypot(goal_.point.x - robot_pose_.position.x, goal_.point.y - robot_pose_.position.y) <= goal_tolerance_;
}

bool PlannerNode::worldToGrid(double x, double y, CellIndex & index) const {
  index.x = static_cast<int>(std::floor((x - current_map_.info.origin.position.x) / current_map_.info.resolution));
  index.y = static_cast<int>(std::floor((y - current_map_.info.origin.position.y) / current_map_.info.resolution));
  return index.x >= 0 && index.x < static_cast<int>(current_map_.info.width) &&
         index.y >= 0 && index.y < static_cast<int>(current_map_.info.height);
}

geometry_msgs::msg::PoseStamped PlannerNode::gridToPose(const CellIndex & index) const {
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = current_map_.header.frame_id.empty() ? "sim_world" : current_map_.header.frame_id;
  pose.pose.position.x =
    current_map_.info.origin.position.x + (static_cast<double>(index.x) + 0.5) * current_map_.info.resolution;
  pose.pose.position.y =
    current_map_.info.origin.position.y + (static_cast<double>(index.y) + 0.5) * current_map_.info.resolution;
  pose.pose.orientation.w = 1.0;
  return pose;
}

bool PlannerNode::isTraversable(const CellIndex & index) const {
  if (index.x < 0 || index.x >= static_cast<int>(current_map_.info.width) ||
      index.y < 0 || index.y >= static_cast<int>(current_map_.info.height)) {
    return false;
  }

  const int8_t value = current_map_.data[index.y * current_map_.info.width + index.x];
  return value >= 0 && value < occupied_threshold_;
}

std::vector<CellIndex> PlannerNode::getNeighbors(const CellIndex & index) const {
  std::vector<CellIndex> neighbors;
  neighbors.reserve(8);
  for (int dy = -1; dy <= 1; ++dy) {
    for (int dx = -1; dx <= 1; ++dx) {
      if (dx == 0 && dy == 0) {
        continue;
      }
      CellIndex neighbor{index.x + dx, index.y + dy};
      if (neighbor.x >= 0 && neighbor.x < static_cast<int>(current_map_.info.width) &&
          neighbor.y >= 0 && neighbor.y < static_cast<int>(current_map_.info.height)) {
        neighbors.push_back(neighbor);
      }
    }
  }
  return neighbors;
}

bool PlannerNode::canMoveBetween(const CellIndex & from, const CellIndex & to) const {
  const int dx = to.x - from.x;
  const int dy = to.y - from.y;
  if (std::abs(dx) != 1 || std::abs(dy) != 1) {
    return true;
  }

  return isTraversable({from.x + dx, from.y}) && isTraversable({from.x, from.y + dy});
}

double PlannerNode::heuristic(const CellIndex & a, const CellIndex & b) const {
  return std::hypot(a.x - b.x, a.y - b.y);
}

std::vector<CellIndex> PlannerNode::reconstructPath(
  const std::unordered_map<int, CellIndex> & came_from, const CellIndex & current) const {
  std::vector<CellIndex> path{current};
  CellIndex cursor = current;
  auto it = came_from.find(cellKey(cursor));
  while (it != came_from.end()) {
    cursor = it->second;
    path.push_back(cursor);
    it = came_from.find(cellKey(cursor));
  }
  std::reverse(path.begin(), path.end());
  return path;
}

int PlannerNode::cellKey(const CellIndex & index) const {
  return index.y * static_cast<int>(current_map_.info.width) + index.x;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
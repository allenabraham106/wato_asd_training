#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "planner_core.hpp"
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <unordered_map>
#include <vector>

struct CellIndex {
    int x = 0;
    int y = 0;
    bool operator==(const CellIndex & o) const { return x == o.x && y == o.y; }
};

struct AStarNode {
    CellIndex index;
    double f_score;
    bool operator>(const AStarNode & o) const { return f_score > o.f_score; }
};

struct CompareF {
    bool operator()(const AStarNode & a, const AStarNode & b) const { return a.f_score > b.f_score; }
};

class PlannerNode : public rclcpp::Node {
public:
    PlannerNode();

private:
    robot::PlannerCore planner_;

    enum class State { WAITING_FOR_GOAL, WAITING_FOR_ROBOT_TO_REACH_GOAL };
    State state_;

    bool have_map_ = false;
    bool have_odom_ = false;
    bool have_goal_ = false;

    double goal_tolerance_ = 0.35;
    double replan_period_s_ = 1.0;
    double cost_weight_ = 0.08;
    int occupied_threshold_ = 65;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    nav_msgs::msg::OccupancyGrid current_map_;
    geometry_msgs::msg::PointStamped goal_;
    geometry_msgs::msg::Pose robot_pose_;

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void timerCallback();
    void planPath();
    void publishEmptyPath();
    bool goalReached() const;
    bool worldToGrid(double x, double y, CellIndex & index) const;
    geometry_msgs::msg::PoseStamped gridToPose(const CellIndex & index) const;
    bool isTraversable(const CellIndex & index) const;
    std::vector<CellIndex> getNeighbors(const CellIndex & index) const;
    bool canMoveBetween(const CellIndex & from, const CellIndex & to) const;
    double heuristic(const CellIndex & a, const CellIndex & b) const;
    std::vector<CellIndex> reconstructPath(const std::unordered_map<int, CellIndex> & came_from, const CellIndex & current) const;
    int cellKey(const CellIndex & index) const;
};

#endif
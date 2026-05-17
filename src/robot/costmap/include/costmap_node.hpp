#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_
 
#include "rclcpp/rclcpp.hpp"
 
#include "costmap_core.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <vector>
 
class CostmapNode : public rclcpp::Node {
  public:
    CostmapNode();
    
    // Place callback function here
    void laserCallBack(const sensor_msgs::msg::LaserScan::SharedPtr scan);
    void initializeCostmap();
    void convertToGrid(double range, double angle, int& x, int& y);
    void markObstacle(int x_grid, int y_grid);
    void inflateObstacles();
    void publishCostmap();
 
  private:
    double resolution_ = 0.1;
    int height_ = 100; 
    int width_ = 100;
    double inflation_radius_ = 3.0;
    int max_cost_ = 90;
    robot::CostmapCore costmap_;
    std::vector<std::vector<int>> costmap_grid_;
    // Place these constructs here
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
};
 
#endif 
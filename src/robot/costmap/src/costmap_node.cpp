#include <chrono>
#include <memory>
#include <cmath>
 
#include "costmap_node.hpp"
 
CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  // Initialize the constructs and their parameters
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar",
    10,
    std::bind(&CostmapNode::laserCallBack, this, std::placeholders::_1)
  );
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
}

void CostmapNode::initializeCostmap(){
  costmap_grid_ = std::vector<std::vector<int>>(height_, std::vector<int>(width_, 0));
}

void CostmapNode::convertToGrid(double range, double angle, int& x_grid, int& y_grid){
  double x = range * cos(angle);
  double y = range * sin(angle);
  x_grid = (x / resolution_) + width_/2;
  y_grid = (y / resolution_) + height_/2;
}

void CostmapNode::markObstacle(int x_grid, int y_grid){
  if (x_grid >= 0 && x_grid < width_ && y_grid >= 0 && y_grid < height_){
    costmap_grid_[y_grid][x_grid] = 100;
  }
}

void CostmapNode::inflateObstacles(){
  for(int i = 0; i < height_; ++i){
    for(int j = 0; j < width_; ++j){
      if(costmap_grid_[i][j] == 100){
        int inflation_cells = inflation_radius_ / resolution_;
        for(int di = -inflation_cells; di <= inflation_cells ; ++di){
          for(int dj = -inflation_cells; dj <= inflation_cells; ++dj){
            double distance = std::sqrt(di * di + dj * dj);
            double distance_meters = distance * resolution_;
            if(distance_meters < inflation_radius_){
              int cost = max_cost_ * (1 - (distance_meters / inflation_radius_));
               if( i + di >= 0 && i + di < height_ && j + dj >= 0 && j + dj < width_){
                if( cost > costmap_grid_[i + di][j + dj]){
                  costmap_grid_[i + di][j + dj] = cost;
                }
              }
            }
          }
        }
      }
    }
  }
}

void CostmapNode::publishCostmap(){
    nav_msgs::msg::OccupancyGrid msg;
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "robot/chassis/lidar";
    msg.info.resolution = resolution_;
    msg.info.width = width_;
    msg.info.height = height_;
    msg.info.origin.position.x = -(width_ * resolution_ / 2.0);
    msg.info.origin.position.y = -(height_ * resolution_ / 2.0);
    msg.info.origin.orientation.w = 1.0;
    std::vector<int8_t> flat;
    for(int i = 0; i < height_; ++i){
        for(int j = 0; j < width_; ++j){
            flat.push_back(costmap_grid_[i][j]);
        }
    }
    msg.data = flat;
    costmap_pub_->publish(msg);
}

void CostmapNode::laserCallBack(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    // Step 1: Initialize costmap
    initializeCostmap();
 
    // Step 2: Convert LaserScan to grid and mark obstacles
    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        double angle = scan->angle_min + i * scan->angle_increment;
        double range = scan->ranges[i];
        if (range < scan->range_max && range > scan->range_min) {
            // Calculate grid coordinates
            int x_grid, y_grid;
            convertToGrid(range, angle, x_grid, y_grid);
            markObstacle(x_grid, y_grid);
        }
    }
 
    // Step 3: Inflate obstacles
    inflateObstacles();
 
    // Step 4: Publish costmap
    publishCostmap();
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}
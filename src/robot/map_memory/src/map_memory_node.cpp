#include "map_memory_node.hpp"
#include <cmath>

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {
        // Initialize subscribers
        initializeMap();
        costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));
 
        // Initialize publisher
        map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
 
        // Initialize timer
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&MapMemoryNode::updateMap, this));
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
    // Store the latest costmap
        latest_costmap_ = *msg;
        costmap_updated_ = true;
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg){
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;
 
        // Compute distance traveled
        double distance = std::sqrt(std::pow(x - last_x_, 2) + std::pow(y - last_y_, 2));
        if (distance >= distance_threshold_) {
            last_x_ = x;
            last_y_ = y;
            should_update_map_ = true;
        }
}

void MapMemoryNode::updateMap(){
    if (should_update_map_ && costmap_updated_) {
        integrateCostmap();
        map_pub_->publish(global_map_);
         should_update_map_ = false;
    }
}

void MapMemoryNode::initializeMap(){
    global_map_.info.resolution = 0.1;
    global_map_.info.width = 300;
    global_map_.info.height = 300;
    global_map_.data = std::vector<int8_t>(300 * 300, 0);
    global_map_.header.frame_id = "sim_world";
    global_map_.info.origin.position.x = -15.0;
    global_map_.info.origin.position.y = -15.0;
    global_map_.info.origin.orientation.w = 1.0;
}

void MapMemoryNode::integrateCostmap(){
    // Transform and merge the latest costmap into the global map
    // (Implementation would handle grid alignment and merging logic)
    int robot_grid_x = (last_x_ / 0.1) + 150;
    int robot_grid_y = (last_y_ / 0.1) + 150;
    for(int i = 0; i < 100; ++i){
      for(int j = 0; j < 100; ++j){
        int global_x = robot_grid_x + (j - 50);
        int global_y = robot_grid_y + (i - 50);
        if(global_x >= 0 && global_x < 300 && global_y >= 0 && global_y < 300){
          int costmap_value = latest_costmap_.data[i * latest_costmap_.info.width + j];
          if(costmap_value != -1){
            global_map_.data[global_y * global_map_.info.width + global_x] = costmap_value;
          }
        }
      }
    }

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}

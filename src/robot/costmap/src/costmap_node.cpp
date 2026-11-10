#include <chrono>
#include <memory>

#include "costmap_node.hpp"

CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  costmap_pub_=this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap",10);
  lidar_sub_=this->create_subscription<sensor_msgs::msg::LaserScan>("/lidar",10,std::bind(&CostmapNode::laserCallback,this,std::placeholders::_1));
}

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  RCLCPP_INFO(this->get_logger(), "Got a scan with %d beams", static_cast<int>(scan->ranges.size()));
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}
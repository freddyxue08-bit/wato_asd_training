#include <chrono>
#include <memory>
#include <cmath>
#include <vector>
#include "costmap_node.hpp"

CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  costmap_pub_=this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap",10);
  lidar_sub_=this->create_subscription<sensor_msgs::msg::LaserScan>("/lidar",10,std::bind(&CostmapNode::laserCallback,this,std::placeholders::_1));
}

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  // ---- Settings ----
  const double resolution = 0.1;   // meters per cell
  const int width = 300;           // cells across
  const int height = 300;          // cells up
  const double origin_x = -15.0;   // bottom-left corner, meters
  const double origin_y = -15.0;
  const double inflation_radius = 1.0;

  // ---- 1. Start a blank grid ----
  nav_msgs::msg::OccupancyGrid grid;
  grid.header = scan->header;
  grid.info.resolution = resolution;
  grid.info.width = width;
  grid.info.height = height;
  grid.info.origin.position.x = origin_x;
  grid.info.origin.position.y = origin_y;
  grid.data.assign(width * height, 0);
  std::vector<std::pair<int, int>> walls;


  // ---- 2. Go through every beam ----
  for (size_t i = 0; i < scan->ranges.size(); i++) {
    double angle = scan->angle_min+i*scan->angle_increment;
    double range = scan->ranges[i];

    // skip junk readings
    if (range < scan->range_min || range > scan->range_max) {
      continue;
    }

    double x =cos(angle)*range;
    double y =sin(angle)*range;

    int col = static_cast<int>(std::floor((x-origin_x)/resolution));
    int row = static_cast<int>(std::floor((y-origin_y)/resolution));

    if (col < 0 || col >= width || row < 0 || row >= height) {
    continue;
}



    int index = row*width+col;
    grid.data[index] = 100;
    walls.push_back({col, row}); 
  }
    // ---- 3. Inflate around every wall cell ----
  const int r_cells = static_cast<int>(inflation_radius / resolution);   // 1.0 / 0.1 = 10 cells

  for (const auto& w : walls) {
    int wc = w.first;    // wall column
    int wr = w.second;   // wall row

    for (int dr = -r_cells; dr <= r_cells; dr++) {
      for (int dc = -r_cells; dc <= r_cells; dc++) {
        int c = wc + dc;   // the nearby cell's column
        int r = wr + dr;   // the nearby cell's row

        // TODO 1: skip if (c, r) is outside the grid
        if (c<0 || c>= width || r<0 || r>=height){
          continue;
        }

        double dist = std::sqrt(dc * dc + dr * dr) * resolution;   // meters

        // TODO 2: skip if dist is bigger than inflation_radius
        if (dist>inflation_radius){
          continue;
        }

        int cost = static_cast<int>(100*(1-dist/inflation_radius));
        int index = r*width+c;

        // TODO 5: only write cost into grid.data[index] if it's BIGGER than what's already there
        if (grid.data[index]>cost){
          continue;
        }
        grid.data[index]=cost;
      }
    }
  }

  // ---- 3. Send it ----
  costmap_pub_->publish(grid);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}
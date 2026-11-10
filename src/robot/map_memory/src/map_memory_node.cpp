#include <chrono>
#include <cmath>
#include <memory>

#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {
  // ---- handles ----
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/costmap", 10,
      std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10,
      std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));

  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

  timer_ = this->create_wall_timer(std::chrono::milliseconds(1000),
                                   std::bind(&MapMemoryNode::updateMap, this));

  // ---- the global map: 100 m x 100 m, 0.2 m cells, pinned to the world ----
  global_map_.header.frame_id = "sim_world";
  global_map_.info.resolution = 0.2;
  global_map_.info.width = 500;              // 500 x 0.2 = 100 m
  global_map_.info.height = 500;
  global_map_.info.origin.position.x = -50.0;
  global_map_.info.origin.position.y = -50.0;
  global_map_.data.assign(500 * 500, 0);
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  latest_costmap_ = *msg;     // copy the whole costmap into memory
  have_costmap_ = true;
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;

  // yaw from the quaternion (standard recipe)
  auto q = msg->pose.pose.orientation;
  robot_yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z));

  have_odom_ = true;
}

void MapMemoryNode::updateMap() {
  if (have_costmap_ && have_odom_) {

    // ---- how far did the robot move since the last paste? ----
    double dx = robot_x_ - last_x_;
    double dy = robot_y_ - last_y_;
    double moved = sqrt(dx*dx+dy*dy);                        // BLANK 1: Pythagoras (std::sqrt)

    if (first_update_ || moved >= 1.5) {

      // ---- short nicknames for the costmap's settings ----
      const auto& cm = latest_costmap_;
      const double c_res = cm.info.resolution;          // 0.1
      const double c_ox = cm.info.origin.position.x;    // -15
      const double c_oy = cm.info.origin.position.y;    // -15
      const int c_w = cm.info.width;                     // 300
      const int c_h = cm.info.height;                    // 300

      // ---- ...and for the global map's settings ----
      const double g_res = global_map_.info.resolution;          // 0.2
      const double g_ox = global_map_.info.origin.position.x;    // -50
      const double g_oy = global_map_.info.origin.position.y;    // -50
      const int g_w = global_map_.info.width;                     // 500
      const int g_h = global_map_.info.height;                    // 500

      const double cos_yaw = std::cos(robot_yaw_);
      const double sin_yaw = std::sin(robot_yaw_);

      // ---- go through every costmap cell ----
      for (int row = 0; row < c_h; row++) {
        for (int col = 0; col < c_w; col++) {
          int cost = cm.data[row * c_w + col];
          if (cost <= 0) {
            continue;                   // nothing to add here
          }

          // STEP 1: costmap cell -> robot frame (meters, cell center)
          double lx = (col+0.5)*c_res+c_ox;            // BLANK 2: uses c_ox, col, c_res (+0.5!)
          double ly = (row+0.5)*c_res+c_oy;            // BLANK 3: uses c_oy, row, c_res

          // STEP 2: robot frame -> world (rotate + add)
          double wx = robot_x_+lx*cos_yaw-ly*sin_yaw;            // BLANK 4: robot_x_, lx, ly, cos_yaw, sin_yaw
          double wy = robot_y_+lx*sin_yaw+ly*cos_yaw;            // BLANK 5: robot_y_, lx, ly, cos_yaw, sin_yaw

          // STEP 3: world -> global map cell
          int gc = static_cast<int>(std::floor((wx-g_ox)/g_res));   // BLANK 6: uses wx, g_ox, g_res
          int gr = static_cast<int>(std::floor((wy-g_oy)/g_res));   // BLANK 7: uses wy, g_oy, g_res

          // STEP 4: skip if outside the global map
          if (gc<0 || g_w<=gc || gr<0 || g_h<=gr) {                  // BLANK 8: four checks with ||, using g_w and g_h
            continue;
          }

          // STEP 5: keep the bigger value
          int gi = gr*g_w+gc;               // BLANK 9: theater seats, with gr, gc, g_w
          if (cost > global_map_.data[gi]) {
            global_map_.data[gi] = cost;
          }
        }
      }

      // ---- remember where we pasted ----
      last_x_ = robot_x_;
      last_y_ = robot_y_;
      first_update_ = false;
    }
  }

  global_map_.header.stamp = this->now();
  map_pub_->publish(global_map_);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}

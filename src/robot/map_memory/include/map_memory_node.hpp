#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

#include "map_memory_core.hpp"

class MapMemoryNode : public rclcpp::Node {
  public:
    MapMemoryNode();
    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void updateMap();

  private:
    robot::MapMemoryCore map_memory_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
        // ---- what the node remembers ----
    nav_msgs::msg::OccupancyGrid latest_costmap_;   // newest costmap
    nav_msgs::msg::OccupancyGrid global_map_;       // the big remembered map
    bool have_costmap_ = false;                      // got at least one costmap yet?
    bool have_odom_ = false;                         // got at least one position yet?

    double robot_x_ = 0.0;     // where the robot is now (world, meters)
    double robot_y_ = 0.0;
    double robot_yaw_ = 0.0;   // which way it faces (radians)

    double last_x_ = 0.0;      // where it was at the last paste
    double last_y_ = 0.0;
    bool first_update_ = true; // paste right away the very first time
};

#endif 

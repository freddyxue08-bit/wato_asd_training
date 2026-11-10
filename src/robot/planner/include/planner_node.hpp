#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"                          // BLANK 1: include for geometry_msgs::msg::PointStamped

#include "planner_core.hpp"

class PlannerNode : public rclcpp::Node {
  public:
    PlannerNode();
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void timerCallback();
    void planPath();

  private:
    robot::PlannerCore planner_;

    // ---- handles ----
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;     // BLANK 2
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;    // BLANK 3
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;    // BLANK 4
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;       // BLANK 5
    rclcpp::TimerBase::SharedPtr timer_;                                         // BLANK 6

    // ---- what the node remembers ----
    nav_msgs::msg::OccupancyGrid map_;
    bool have_map_ = false;

    geometry_msgs::msg::PointStamped goal_;
    bool have_goal_ = false;     // false = waiting for a goal, true = working on it

    double robot_x_ = 0.0;
    double robot_y_ = 0.0;
    bool have_odom_ = false;
};

#endif
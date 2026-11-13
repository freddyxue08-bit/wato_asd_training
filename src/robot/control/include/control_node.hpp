#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "control_core.hpp"

class ControlNode : public rclcpp::Node {
  public:
    ControlNode();
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void controlLoop();

  private:
    robot::ControlCore control_;

    // ---- handles ----
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // ---- what the node remembers ----
    nav_msgs::msg::Path path_;
    bool have_path_ = false;

    double robot_x_ = 0.0;
    double robot_y_ = 0.0;
    double robot_yaw_ = 0.0;
    bool have_odom_ = false;

    // ---- tuning knobs ----
    const double lookahead_ = 1.0;        // meters: how far ahead the carrot is
    const double linear_speed_ = 0.5;     // m/s forward
    const double max_angular_ = 1.0;      // rad/s: never turn faster than this
    const double goal_tolerance_ = 0.3;   // meters: close enough to the end, stop
};

#endif

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>

#include "control_node.hpp"

ControlNode::ControlNode() : Node("control"), control_(robot::ControlCore(this->get_logger())) {
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/path", 10,
      std::bind(&ControlNode::pathCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10,
      std::bind(&ControlNode::odomCallback, this, std::placeholders::_1));

  cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  timer_ = this->create_wall_timer(std::chrono::milliseconds(100),   // 10 times a second
                                   std::bind(&ControlNode::controlLoop, this));
}

void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
  path_ = *msg;
  have_path_ = !msg->poses.empty();   // an empty path means "stop"
}

void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_x_ = msg->pose.pose.position.x;     // BLANK 1
  robot_y_ = msg->pose.pose.position.y;     // BLANK 2

  auto q = msg->pose.pose.orientation;
  robot_yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  have_odom_ = true;
}

void ControlNode::controlLoop() {
  geometry_msgs::msg::Twist cmd;   // starts as all zeros = "stand still"

  if (!have_path_ || !have_odom_) {
    cmd_pub_->publish(cmd);        // nothing to follow: stay still
    return;
  }

  // ---- 1. close enough to the end of the path? stop ----
  const auto& last = path_.poses.back().pose.position;
  double dist_to_end = std::hypot(last.x - robot_x_, last.y - robot_y_);
  if (dist_to_end < goal_tolerance_) {
    cmd_pub_->publish(cmd);
    return;
  }

  // ---- 2. find the carrot: the first path point at least lookahead_ away ----
  double cx = last.x;              // if none is far enough, use the last point
  double cy = last.y;
  for (const auto& ps : path_.poses) {
    double px = ps.pose.position.x;
    double py = ps.pose.position.y;
    double d = sqrt((robot_x_-px)*(robot_x_-px)+(robot_y_-py)*(robot_y_-py));               // BLANK 3: distance from robot to (px, py)  (std::hypot or sqrt)
    if (d >= lookahead_) {
      cx = px;
      cy = py;
      break;                       // found it, stop looking
    }
  }

  // ---- 3. carrot: world frame -> robot frame (Map Memory backwards) ----
  double dx = cx - robot_x_;
  double dy = cy - robot_y_;
  double local_x =  dx * std::cos(robot_yaw_) + dy * std::sin(robot_yaw_);
  double local_y = -dx * std::sin(robot_yaw_) + dy * std::cos(robot_yaw_);

  // ---- 4. choose the speeds ----
  if (local_x < 0.0) {
    // carrot is behind us: turn in place toward it
    cmd.linear.x = 0.0;
    cmd.angular.z = (local_y >= 0.0) ? max_angular_ : -max_angular_;
  } else {
    double dist2 = local_x * local_x + local_y * local_y;
    cmd.linear.x = linear_speed_;
    cmd.angular.z = 2*linear_speed_*local_y/dist2;          // BLANK 6: the Pure Pursuit formula
    cmd.angular.z = std::clamp(cmd.angular.z, -max_angular_, max_angular_);   // never turn too hard
  }

  cmd_pub_->publish(cmd);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}

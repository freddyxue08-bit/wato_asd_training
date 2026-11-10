#include <chrono>
#include <cmath>
#include <memory>

#include "planner_node.hpp"

PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())) {
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", 10,                                                       // BLANK 1: map topic
      std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));

  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/goal_point", 10,                                                       // BLANK 2: goal topic
      std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10,                                                       // BLANK 3: position topic
      std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);   // BLANK 4: path topic

  timer_ = this->create_wall_timer(std::chrono::milliseconds(500),      // BLANK 5: half a second
                                   std::bind(&PlannerNode::timerCallback, this));
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  map_ = *msg;
  have_map_ = true;
  if (have_goal_) {
    planPath();          // the map changed, so replan
  }
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  goal_ = *msg;
  have_goal_ = true;
  RCLCPP_INFO(this->get_logger(), "New goal: (%.2f, %.2f) in frame '%s'",
              goal_.point.x, goal_.point.y, goal_.header.frame_id.c_str());
  planPath();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_x_ =msg->pose.pose.position.x;       // BLANK 6
  robot_y_ =msg->pose.pose.position.y;       // BLANK 7
  have_odom_ = true;
}

void PlannerNode::timerCallback() {
  if (!have_goal_ || !have_odom_) {
    return;              // nothing to check yet
  }

  double dx = goal_.point.x - robot_x_;
  double dy = goal_.point.y - robot_y_;
  double dist = sqrt(dx*dx+dy*dy);    // BLANK 8: Pythagoras

  if (dist<0.5) {            // BLANK 9: closer than 0.5 m?
    RCLCPP_INFO(this->get_logger(), "Goal reached!");
    have_goal_ = false;  // back to waiting for a goal

    nav_msgs::msg::Path empty;            // an empty path means "stop"
    empty.header.frame_id = "sim_world";
    empty.header.stamp = this->now();
    path_pub_->publish(empty);
  }
}

void PlannerNode::planPath() {
  if (!have_odom_) {
    RCLCPP_WARN(this->get_logger(), "No robot position yet, can't plan");
    return;
  }

  nav_msgs::msg::Path path;
  path.header.frame_id = "sim_world";
  path.header.stamp = this->now();

  // ---- TEMPORARY: a straight line from robot to goal (A* replaces this next lesson) ----
  for (int i = 0; i <= 20; i++) {
    double t = i / 20.0;                  // 0.0, 0.05, 0.10 ... 1.0 along the line
    geometry_msgs::msg::PoseStamped p;
    p.header = path.header;
    p.pose.position.x = robot_x_ + t * (goal_.point.x - robot_x_);
    p.pose.position.y = robot_y_ + t * (goal_.point.y - robot_y_);
    path.poses.push_back(p);
  }

  path_pub_->publish(path);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
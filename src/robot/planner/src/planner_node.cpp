#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <queue>
#include <vector>

#include "planner_node.hpp"

PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())) {
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", 10,
      std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));

  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/goal_point", 10,
      std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10,
      std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

  timer_ = this->create_wall_timer(std::chrono::milliseconds(500),
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
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;
  have_odom_ = true;
}

void PlannerNode::timerCallback() {
  if (!have_goal_ || !have_odom_) {
    return;              // nothing to check yet
  }

  double dx = goal_.point.x - robot_x_;
  double dy = goal_.point.y - robot_y_;
  double dist = std::sqrt(dx * dx + dy * dy);

  if (dist < 0.5) {
    RCLCPP_INFO(this->get_logger(), "Goal reached!");
    have_goal_ = false;  // back to waiting for a goal

    nav_msgs::msg::Path empty;            // an empty path means "stop"
    empty.header.frame_id = "sim_world";
    empty.header.stamp = this->now();
    path_pub_->publish(empty);
  }
}

void PlannerNode::planPath() {
  if (!have_odom_ || !have_map_) {
    RCLCPP_WARN(this->get_logger(), "Need a map and a position before planning");
    return;
  }

  // ---- map settings (nicknames) ----
  const double res = map_.info.resolution;
  const double ox = map_.info.origin.position.x;
  const double oy = map_.info.origin.position.y;
  const int W = map_.info.width;
  const int H = map_.info.height;

  // ---- 1. start and goal: world meters -> cells (Lesson 6) ----
  int sc = static_cast<int>(std::floor((robot_x_-ox)/res));   // BLANK 1: robot_x_, ox, res
  int sr = static_cast<int>(std::floor((robot_y_-oy)/res));   // BLANK 2: robot_y_, oy, res
  int gc = static_cast<int>(std::floor((goal_.point.x-ox)/res));   // BLANK 3: goal_.point.x, ox, res
  int gr = static_cast<int>(std::floor((goal_.point.y-oy)/res));   // BLANK 4: goal_.point.y, oy, res

  if (sc < 0 || sc >= W || sr < 0 || sr >= H || gc < 0 || gc >= W || gr < 0 || gr >= H) {
    RCLCPP_WARN(this->get_logger(), "Start or goal is outside the map");
    return;
  }

  const int start = sr*W+sc;   // BLANK 5: theater seats, with sr, sc, W
  const int goal = gr*W+gc;    // BLANK 6: theater seats, with gr, gc, W

  const int BLOCKED = 40;   // cost at or above this = can't drive here
  if (map_.data[goal] >= BLOCKED) {
    RCLCPP_WARN(this->get_logger(), "Goal is inside or too close to a wall");
    return;
  }

  // ---- 2. A* memory ----
  const int N = W * H;
  const double INF = std::numeric_limits<double>::infinity();
  std::vector<double> g(N, INF);
  std::vector<int> came_from(N, -1);
  std::vector<bool> closed(N, false);

  using Item = std::pair<double, int>;   // (f, cell index)
  std::priority_queue<Item, std::vector<Item>, std::greater<Item>> open;

  g[start] = 0.0;
  open.push({std::hypot(gc - sc, gr - sr), start});   // f = 0 + h
  bool found = false;

  // ---- 3. the A* loop ----
  while (!open.empty()) {
    int cur = open.top().second;    // smallest f
    open.pop();

    if (closed[cur]) {
      continue;                     // already done (an old, outdated entry)
    }
    if (cur == goal) {
      found = true;
      break;                        // reached the goal, stop searching
    }
    closed[cur] = true;

    int cc = cur % W;               // this cell's column
    int cr = cur / W;               // this cell's row

    for (int dr = -1; dr <= 1; dr++) {
      for (int dc = -1; dc <= 1; dc++) {
        if (dc == 0 && dr == 0) {
          continue;                 // that's the cell itself, not a neighbour
        }
        int nc = cc + dc;
        int nr = cr + dr;

        if (nc>=W || nr>=H || nc<0 || nr<0) {                 // BLANK 7: neighbour outside the map? (nc, nr, W, H)
          continue;
        }
        int ni = nr*W+nc;              // BLANK 8: neighbour's index

        if (closed[ni]) {
          continue;
        }
        int cost = map_.data[ni];
        if (cost >= BLOCKED) {
          continue;                 // too close to a wall
        }

        double step = (dc != 0 && dr != 0) ? 1.414 : 1.0;   // diagonal or straight
        step *= 1.0 + cost / 25.0;                           // stay away from walls

        double new_g = g[cur] + step;
        if (new_g < g[ni]) {        // found a better way to reach this neighbour
          g[ni] = new_g;
          came_from[ni] = cur;
          double h = std::hypot(gc - nc, gr - nr);
          open.push({new_g+h, ni});    // BLANK 9: f = ?
        }
      }
    }
  }

  if (!found) {
    RCLCPP_WARN(this->get_logger(), "No path found");
    return;
  }

  // ---- 4. trace back through came_from, then flip ----
  std::vector<int> cells;
  for (int c = goal; c != -1; c = came_from[c]) {
    cells.push_back(c);
  }
  std::reverse(cells.begin(), cells.end());

  // ---- 5. cells -> a Path message (cell centers, in meters) ----
  nav_msgs::msg::Path path;
  path.header.frame_id = "sim_world";
  path.header.stamp = this->now();

  for (int idx : cells) {
    geometry_msgs::msg::PoseStamped p;
    p.header = path.header;
    p.pose.position.x = ox + (idx % W + 0.5) * res;
    p.pose.position.y = oy + (idx / W + 0.5) * res;   // BLANK 11: oy, row (idx / W), +0.5, res
    path.poses.push_back(p);
  }
  RCLCPP_INFO(this->get_logger(), "Path from (%.2f, %.2f) to (%.2f, %.2f), %d cells",
              path.poses.front().pose.position.x, path.poses.front().pose.position.y,
              path.poses.back().pose.position.x, path.poses.back().pose.position.y,
              static_cast<int>(path.poses.size()));
  path_pub_->publish(path);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}

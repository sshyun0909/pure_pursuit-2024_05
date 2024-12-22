#include <algorithm>
#include <cmath>
#include <fstream>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <limits>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <string>
#include <vector>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/bool.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker.hpp"

class PurePursuit : public rclcpp::Node
{
public:
  PurePursuit()
  : Node("pure_pursuit_node")
  {
    this->declare_parameter<std::string>("odom_topic", "/ego_racecar/odom");
    this->declare_parameter<std::string>("drive_topic", "/drive");
    this->declare_parameter<std::string>("ackermann_cmd_topic", "/ackermann_cmd");
    this->declare_parameter<std::string>("imferred_pose_topic", "/pf/viz/imferred_pose");
    this->declare_parameter("min_lookahead", 0.5);
    this->declare_parameter("max_lookahead", 2.0);
    this->declare_parameter("min_velocity", 1.0);
    this->declare_parameter("max_velocity", 1.0);
    this->declare_parameter("smoothing_factor", 0.8);
    this->declare_parameter("curvature_normalization_factor", 0.5);
    this->declare_parameter<std::string>("global_path_topic", "/global/path");
    this->declare_parameter("Ts", 0.05);

    this->odom_topic = this->get_parameter("odom_topic").as_string();
    this->drive_topic = this->get_parameter("drive_topic").as_string();
    this->ackermann_cmd_topic = this->get_parameter("ackermann_cmd_topic").as_string();
    this->imferred_pose_topic = this->get_parameter("imferred_pose_topic").as_string();
    this->min_lookahead = this->get_parameter("min_lookahead").as_double();
    this->max_lookahead = this->get_parameter("max_lookahead").as_double();
    this->min_velocity = this->get_parameter("min_velocity").as_double();
    this->max_velocity = this->get_parameter("max_velocity").as_double();
    this->smoothing_factor = this->get_parameter("smoothing_factor").as_double();
    this->curvature_normalization_factor =
      this->get_parameter("curvature_normalization_factor").as_double();
    this->global_path_topic = this->get_parameter("global_path_topic").as_string();
    this->Ts = this->get_parameter("Ts").as_double();

    global_path_sub_ =
      this->create_subscription<nav_msgs::msg::Path>(
      "/global/path", 10,
      std::bind(&PurePursuit::global_path_callback, this, std::placeholders::_1));
    brake_sub_ =
      this->create_subscription<std_msgs::msg::Bool>(
      "/emergency_braking", 10,
      std::bind(&PurePursuit::brake_callback, this, std::placeholders::_1));
    drive_pub_timer =
      create_wall_timer(
      std::chrono::duration<double>(Ts),
      std::bind(&PurePursuit::publish_drive_callback, this));

    velocity = 0.0;
    steering_angle = 0.0;
    current_waypoint_index = 0;
    emergency_braking = false;
    odom_published = false;
  }

private:
  double L, min_velocity, max_velocity, smoothing_factor, curvature_normalization_factor;
  double Ts;
  bool odom_published;

  double min_lookahead = 0.5;
  double max_lookahead = 2.5;
  std::string drive_topic, odom_topic, waypoints_file, ackermann_cmd_topic, imferred_pose_topic,
    global_path_topic;
  std::vector<std::vector<double>> waypoints;
  bool emergency_braking;
  double velocity;
  double steering_angle;
  size_t current_waypoint_index;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr visualize_pub_marker_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr race_drive_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr race_pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_sub_;
  bool path_received = false;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr brake_sub_;
  rclcpp::TimerBase::SharedPtr drive_pub_timer;

  void brake_callback(const std_msgs::msg::Bool::SharedPtr data)
  {
    this->emergency_braking = data->data;
  }

  void global_path_callback(const nav_msgs::msg::Path::SharedPtr global_path)
  {
    if (path_received) {
      return;
    }

    for (const auto & pose_stamped : global_path->poses) {
      double x = pose_stamped.pose.position.x;
      double y = pose_stamped.pose.position.y;
      this->waypoints.push_back({y, x});
    }
    path_received = true;

    pose_sub_ =
      this->create_subscription<nav_msgs::msg::Odometry>(
      this->odom_topic, 10,
      std::bind(&PurePursuit::pose_callback_odometry, this, std::placeholders::_1));
    drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
      this->drive_topic, 10);

    visualize_pub_marker_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/goal_waypoint", 10);

    race_drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
      this->ackermann_cmd_topic, 10);
    race_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/pf/viz/inferred_pose", 10,
      std::bind(&PurePursuit::pose_callback_pose_stamped, this, std::placeholders::_1));
  }

  void publish_drive_callback()
  {
    if (!odom_published) {return;}

    publish_drive(velocity, steering_angle);
  }

  void pose_callback_odometry(const nav_msgs::msg::Odometry::SharedPtr pose_msg)
  {
    double car_x = pose_msg->pose.pose.position.x;
    double car_y = pose_msg->pose.pose.position.y;

    this->L = min_lookahead + (this->velocity / this->max_velocity) *
      (max_lookahead - min_lookahead);

    auto goal_waypoint = get_goal_waypoint({car_y, car_x});

    auto q = pose_msg->pose.pose.orientation;
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    double heading_current = std::atan2(siny_cosp, cosy_cosp);

    double euclidean_dist = std::hypot(goal_waypoint[0] - car_y, goal_waypoint[1] - car_x);

    double lookahead_angle = std::atan2(goal_waypoint[0] - car_y, goal_waypoint[1] - car_x);

    double delta_y = euclidean_dist * std::sin(lookahead_angle - heading_current);

    steering_angle = calculate_steering_angle(this->L, delta_y);

    velocity = calculate_velocity(delta_y);

    odom_published = true;
  }

  void pose_callback_pose_stamped(const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg)
  {
    if (!path_received) {
      return;
    }

    double car_x = pose_msg->pose.position.x;
    double car_y = pose_msg->pose.position.y;

    double min_lookahead = 0.8;
    double max_lookahead = 1.0;
    this->L = min_lookahead + (this->velocity / this->max_velocity) *
      (max_lookahead - min_lookahead);

    auto goal_waypoint = get_goal_waypoint({car_y, car_x});

    auto q = pose_msg->pose.orientation;
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    double heading_current = std::atan2(siny_cosp, cosy_cosp);

    double euclidean_dist = std::hypot(goal_waypoint[0] - car_y, goal_waypoint[1] - car_x);

    double lookahead_angle = std::atan2(goal_waypoint[0] - car_y, goal_waypoint[1] - car_x);

    double delta_y = euclidean_dist * std::sin(lookahead_angle - heading_current);

    steering_angle = calculate_steering_angle(this->L, delta_y);

    velocity = calculate_velocity(delta_y);

    odom_published = true;
  }

  std::vector<double> get_goal_waypoint(const std::vector<double> & car_point)
  {
    auto closest_it = std::min_element(
      waypoints.begin(), waypoints.end(),
      [&car_point](const std::vector<double> & a, const std::vector<double> & b) {
        return std::hypot(
          car_point[0] - a[0],
          car_point[1] - a[1]) < std::hypot(car_point[0] - b[0], car_point[1] - b[1]);
      });

    size_t closest_index = std::distance(waypoints.begin(), closest_it);

    size_t i = closest_index;
    while (true) {
      double distance = std::hypot(
        car_point[0] - waypoints[i % waypoints.size()][0],
        car_point[1] - waypoints[i % waypoints.size()][1]);
      if (distance >= this->L) {
        this->current_waypoint_index = i % waypoints.size();
        break;
      }
      i++;
    }

    auto goal_waypoint = waypoints[this->current_waypoint_index];

    visualization_msgs::msg::Marker marker;
    marker.id = -2;
    marker.header.frame_id = "map";
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = goal_waypoint[1];
    marker.pose.position.y = goal_waypoint[0];
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    this->visualize_pub_marker_->publish(marker);

    return goal_waypoint;
  }

  double calculate_steering_angle(double L, double y)
  {
    return (2 * y) / (L * L);
    // return (2 * y) / pow(L, 2.7);
  }

  double calculate_velocity(double delta_y)
  {
    double curvature = std::abs(delta_y) / this->L;
    double normalized_curvature = std::min(curvature / this->curvature_normalization_factor, 1.0);
    double target_velocity = (this->max_velocity * (1.0 - normalized_curvature)) +
      (this->min_velocity * normalized_curvature);
    this->velocity = (this->smoothing_factor * target_velocity) +
      ((1.0 - this->smoothing_factor) * this->velocity);

    if (this->emergency_braking) {
      return 0.0;
    } else {
      return this->velocity;
    }
  }

  void publish_drive(double speed, double angle)
  {
    geometry_msgs::msg::Point lookahead_point;
    auto msg = ackermann_msgs::msg::AckermannDriveStamped();
    msg.drive.speed = speed;
    msg.drive.steering_angle = angle;
    RCLCPP_INFO(this->get_logger(), "speed: %.2f m/s, lookahead: %.2f", speed, L);
    this->drive_pub_->publish(msg);
    this->race_drive_pub_->publish(msg);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PurePursuit>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

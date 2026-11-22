#include <array>
#include <cmath>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

class EightTrajectory : public rclcpp::Node {
public:
  EightTrajectory() : Node("eight_trajectory_node") {

    publisher_wheel_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        "/wheel_speed", 10);

    subscriber_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/filtered", 10,
        std::bind(&EightTrajectory::odom_callback, this,
                  std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
                "Figure eight trajectory node initialized.");
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Use odom to calculate current_yaw_
    tf2::Quaternion q(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    curr_phi_ = static_cast<float>(yaw);
    curr_x_ = msg->pose.pose.position.x;
    curr_y_ = msg->pose.pose.position.y;

    // Initialize target pose using the initial odometry
    if (!initialized_) {
      target_x_ = curr_x_;
      target_y_ = curr_y_;
      target_phi_ = curr_phi_;

      initialized_ = true;
      new_waypoint_ = true;
      RCLCPP_INFO(this->get_logger(), "Current waypoint: %zu",
                  waypoint_index_ + 1);
      return;
    }

    if (trajectory_complete_) {
      std_msgs::msg::Float32MultiArray stop;
      stop.data = {0.0, 0.0, 0.0, 0.0};
      publisher_wheel_->publish(stop);
      return;
    }

    if (new_waypoint_) {
      new_waypoint_ = false;
      target_x_ += waypoints_[waypoint_index_][1];
      target_y_ += waypoints_[waypoint_index_][2];
      target_phi_ += waypoints_[waypoint_index_][0];
    }

    error_x_ = target_x_ - curr_x_;
    error_y_ = target_y_ - curr_y_;
    error_phi_ = std::atan2(std::sin(target_phi_ - curr_phi_),
                            std::cos(target_phi_ - curr_phi_));

    if (std::abs(error_x_) < 0.05 && std::abs(error_y_) < 0.05 &&
        std::abs(error_phi_) < 0.05) {

      RCLCPP_INFO(this->get_logger(),
                  "Waypoint %zu reached | Target: (%.2f, %.2f, %.2f) | "
                  "Actual: (%.2f, %.2f, %.2f)",
                  waypoint_index_ + 1, target_x_, target_y_, target_phi_,
                  curr_x_, curr_y_, curr_phi_);

      if (waypoint_index_ < waypoints_.size() - 1) {
        new_waypoint_ = true;
        waypoint_index_++;
        RCLCPP_INFO(this->get_logger(), "Current waypoint: %zu",
                    waypoint_index_ + 1);
      } else {
        trajectory_complete_ = true;
        RCLCPP_INFO(this->get_logger(), "Trajectory complete!");
      }
      std_msgs::msg::Float32MultiArray stop;
      stop.data = {0.0, 0.0, 0.0, 0.0};
      publisher_wheel_->publish(stop);
      return;
    }

    // 3. Calculate desired world-frame velocities
    std::array<float, 3> world_velocity = {kp_xy * error_x_, kp_xy * error_y_,
                                           kp_phi * error_phi_};

    // 4. Transform world velocities into robot frame
    auto twist_vel = velocity2twist(world_velocity);

    // 5. Convert robot Twist into wheel velocities
    auto wheel_velocities = twist2wheels(twist_vel);

    // 6. Publish wheel velocities
    publisher_wheel_->publish(wheel_velocities);
  }

  std::array<float, 3> velocity2twist(std::array<float, 3> world_velocity) {
    std::array<float, 3> twist_vel;

    twist_vel[0] = std::cos(curr_phi_) * world_velocity[0] +
                   std::sin(curr_phi_) * world_velocity[1];
    twist_vel[1] = -std::sin(curr_phi_) * world_velocity[0] +
                   std::cos(curr_phi_) * world_velocity[1];
    twist_vel[2] = world_velocity[2];

    return twist_vel;
  }

  std_msgs::msg::Float32MultiArray
  twist2wheels(std::array<float, 3> twist_vel) {

    std_msgs::msg::Float32MultiArray wheel_velocities;

    float vx = twist_vel[0];
    float vy = twist_vel[1];
    float wz = twist_vel[2];

    float u1 = (vx - vy - (l_ + w_) * wz) / r_;
    float u2 = (vx + vy + (l_ + w_) * wz) / r_;
    float u3 = (vx - vy + (l_ + w_) * wz) / r_;
    float u4 = (vx + vy - (l_ + w_) * wz) / r_;

    wheel_velocities.data = {u1, u2, u3, u4};

    return wheel_velocities;
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_odom_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr
      publisher_wheel_;

  float curr_x_ = 0.0f;
  float curr_y_ = 0.0f;
  float curr_phi_ = 0.0f;
  float r_ = 0.05f;
  float l_ = 0.085f;
  float w_ = 0.134845f;

  // Waypoints: [dphi, dx, dy]
  std::vector<std::array<float, 3>> waypoints_ = {
      {0.0, 1.0, -1.0},     {0.0, 1.0, 1.0},       {0.0, 1.0, 1.0},
      {-1.5708, 1.0, -1.0}, {-1.5708, -1.0, -1.0}, {0.0, -1.0, 1.0},
      {0.0, -1.0, 1.0},     {0.0, -1.0, -1.0}};

  // Current waypoint
  size_t waypoint_index_ = 0;
  bool new_waypoint_ = true;
  bool initialized_ = false;
  bool trajectory_complete_ = false;

  // Target pose (world frame)
  float target_x_ = 0.0;
  float target_y_ = 0.0;
  float target_phi_ = 0.0;

  // Current error
  float error_x_ = 0.0;
  float error_y_ = 0.0;
  float error_phi_ = 0.0;

  // Error control
  float kp_xy = 0.5;
  float kp_phi = 0.5;
};

int main(int argc, char **argv) {
  // initialize the ROS2 communication
  rclcpp::init(argc, argv);
  // declare the node constructor
  auto node = std::make_shared<EightTrajectory>();
  // keeps the node alive, waits for a request to kill the node (ctrl+c)
  rclcpp::spin(node);
  return 0;
}
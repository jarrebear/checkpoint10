#include <chrono>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/float32_multi_array.hpp>

enum class Direction {
  FORWARD,
  BACKWARD,
  LEFT,
  RIGHT,
  CLOCKWISE,
  COUNTER_CLOCKWISE,
  STOP
};

class WheelVelocities : public rclcpp::Node {
public:
  WheelVelocities() : Node("wheel_velocities_node") {

    publisher_wheel_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        "/wheel_speed", 10);

    RCLCPP_INFO(this->get_logger(),
                "Wheel velocities publisher node initialized.");
  }

  void run_sequence() {

    while (rclcpp::ok() && publisher_wheel_->get_subscription_count() == 0) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Waiting for wheel speed subscriber...");
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    float wheel_speed = 3.14f;
    RCLCPP_INFO(this->get_logger(), "Moving forward");
    execute_movement(wheel_speed, Direction::FORWARD);

    RCLCPP_INFO(this->get_logger(), "Moving backwards");
    execute_movement(wheel_speed, Direction::BACKWARD);

    RCLCPP_INFO(this->get_logger(), "Moving left");
    execute_movement(wheel_speed, Direction::LEFT);

    RCLCPP_INFO(this->get_logger(), "Moving right");
    execute_movement(wheel_speed, Direction::RIGHT);

    RCLCPP_INFO(this->get_logger(), "Moving clockwise");
    execute_movement(wheel_speed, Direction::CLOCKWISE);

    RCLCPP_INFO(this->get_logger(), "Moving counter clockwise");
    execute_movement(wheel_speed, Direction::COUNTER_CLOCKWISE);

    RCLCPP_INFO(this->get_logger(), "Stopping");
    execute_movement(0.0f, Direction::STOP);
  }

private:
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr
      publisher_wheel_;

  float command_execution_time_ = 3.0f;
  std_msgs::msg::Float32MultiArray current_command_;

  void execute_movement(float angular_speed, Direction direction) {

    switch (direction) {
    case Direction::FORWARD:
      current_command_.data = {angular_speed, angular_speed, angular_speed,
                               angular_speed};
      break;

    case Direction::BACKWARD:
      current_command_.data = {-angular_speed, -angular_speed, -angular_speed,
                               -angular_speed};
      break;

    case Direction::LEFT:
      current_command_.data = {-angular_speed, angular_speed, angular_speed,
                               -angular_speed};
      break;

    case Direction::RIGHT:
      current_command_.data = {angular_speed, -angular_speed, -angular_speed,
                               angular_speed};
      break;

    case Direction::CLOCKWISE:
      current_command_.data = {angular_speed, -angular_speed, angular_speed,
                               -angular_speed};
      break;

    case Direction::COUNTER_CLOCKWISE:
      current_command_.data = {-angular_speed, angular_speed, -angular_speed,
                               angular_speed};
      break;

    case Direction::STOP:
      current_command_.data = {0.0f, 0.0f, 0.0f, 0.0f};
      publisher_wheel_->publish(current_command_);
      return;

    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown movement direction");
      return;
    }

    auto start_time = std::chrono::steady_clock::now();

    while (std::chrono::steady_clock::now() - start_time <
           std::chrono::duration<float>(command_execution_time_)) {

      publisher_wheel_->publish(current_command_);

      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Stop the wheels after the movement.
    current_command_.data = {0.0f, 0.0f, 0.0f, 0.0f};
    publisher_wheel_->publish(current_command_);
  }
};

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);

  auto node = std::make_shared<WheelVelocities>();
  node->run_sequence();
  rclcpp::shutdown();

  return 0;
}
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

class KinematicModel : public rclcpp::Node {
public:
  KinematicModel() : Node("kinematics_node") {
    publisher_vel_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    subscriber_wheel_speed_ =
        this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/wheel_speed", 10,
            std::bind(&KinematicModel::speed_callback, this,
                      std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Kinematic model node initialized.");
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_vel_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr
      subscriber_wheel_speed_;

  float r_ = 0.05f;
  float l_ = 0.085f;
  float w_ = 0.134845f;

  void speed_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {

    geometry_msgs::msg::Twist velocity_command;

    velocity_command = wheelvelocity2twist(*msg);

    publisher_vel_->publish(velocity_command);
  }

  geometry_msgs::msg::Twist
  wheelvelocity2twist(const std_msgs::msg::Float32MultiArray &current_command) {

    geometry_msgs::msg::Twist twist_velocity;
    twist_velocity.linear.x =
        r_ *
        (current_command.data[0] + current_command.data[1] +
         current_command.data[2] + current_command.data[3]) /
        4;

    twist_velocity.linear.y =
        r_ *
        (-current_command.data[0] + current_command.data[1] -
         current_command.data[2] + current_command.data[3]) /
        4;

    twist_velocity.angular.z =
        r_ *
        (-current_command.data[0] + current_command.data[1] +
         current_command.data[2] - current_command.data[3]) /
        (4 * (l_ + w_));

    return twist_velocity;
  }
};

int main(int argc, char **argv) {
  // initialize the ROS2 communication
  rclcpp::init(argc, argv);
  // declare the node constructor
  auto node = std::make_shared<KinematicModel>();
  // keeps the node alive, waits for a request to kill the node (ctrl+c)
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
#include <chrono>
#include <cmath>
#include <memory>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;

class FlightDynamicsNode : public rclcpp::Node
{
public:
  FlightDynamicsNode()
  : Node("flight_dynamics_node"),
    forward_speed_(3.5),
    pitch_angle_(0.0),
    pitch_rate_(0.0),
    elevator_deflection_(0.0),
    touchdown_(false)
  {
    this->declare_parameter<double>("cruise_speed", 3.5);
    forward_speed_ = this->get_parameter("cruise_speed").as_double();

    elevator_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      "/control/elevator_cmd",
      10,
      [this](const std_msgs::msg::Float64::SharedPtr msg) {
        elevator_deflection_ = msg->data;
      }
    );

    attitude_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
      "/control/attitude",
      10,
      [this](const geometry_msgs::msg::Vector3::SharedPtr msg) {
        pitch_angle_ = msg->y;
      }
    );

    touchdown_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/flight/touchdown",
      10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        touchdown_ = msg->data;
      }
    );

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/flight/cmd_vel",
      10
    );

    last_time_ = this->now();
    timer_ = this->create_wall_timer(
      20ms,
      std::bind(&FlightDynamicsNode::update_dynamics, this)
    );

    RCLCPP_INFO(this->get_logger(), "Flight Dynamics active. Streaming 3.50 m/s cruise velocity.");
  }

private:
  void update_dynamics()
  {
    rclcpp::Time now = this->now();
    double dt = (now - last_time_).seconds();
    last_time_ = now;

    if (dt <= 0.0 || dt > 0.1) {
      dt = 0.02;
    }

    auto twist = geometry_msgs::msg::Twist();

    if (touchdown_) {
      // Landed: halt all motion
      cmd_vel_pub_->publish(twist);
      return;
    }

    // Aerodynamic pitching moment from stabilator
    constexpr double K_elevator = -4.5;
    constexpr double K_damping  =  3.0;

    double pitch_accel = (K_elevator * elevator_deflection_) - (K_damping * pitch_rate_);
    pitch_rate_ += pitch_accel * dt;
    pitch_rate_ = std::clamp(pitch_rate_, -1.2, 1.2);

    // Glideslope sink rate: controlled descent rate of -0.06 m/s
    double v_z = (forward_speed_ * std::sin(pitch_angle_)) - 0.06;

    twist.linear.x = forward_speed_ * std::cos(pitch_angle_);
    twist.linear.y = 0.0;
    twist.linear.z = v_z;

    twist.angular.x = 0.0;
    twist.angular.y = pitch_rate_;
    twist.angular.z = 0.0;

    cmd_vel_pub_->publish(twist);
  }

  double forward_speed_;
  double pitch_angle_;
  double pitch_rate_;
  double elevator_deflection_;
  bool touchdown_;

  rclcpp::Time last_time_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr elevator_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr attitude_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr touchdown_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FlightDynamicsNode>());
  rclcpp::shutdown();
  return 0;
}

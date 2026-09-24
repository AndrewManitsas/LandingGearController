#include <chrono>
#include <cmath>
#include <memory>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;

class LandingGearController : public rclcpp::Node
{
public:
  LandingGearController()
  : Node("landing_gear_controller"),
    current_position_(0.0),
    target_position_(0.0),
    deploy_requested_(false),
    abort_active_(false)
  {
    this->declare_parameter<double>("stowed_angle", 0.0);
    this->declare_parameter<double>("deployed_angle", 1.5708);
    this->declare_parameter<double>("rate_limit", 0.50);  // rad/s

    stowed_angle_ = this->get_parameter("stowed_angle").as_double();
    deployed_angle_ = this->get_parameter("deployed_angle").as_double();
    rate_limit_ = this->get_parameter("rate_limit").as_double();

    // 1. Operator / Flight Stack Gear Deploy Request
    deploy_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/control/gear_deploy",
      10,
      std::bind(&LandingGearController::deploy_callback, this, std::placeholders::_1)
    );

    // 2. Edge AI Safety Interlock (Phase 4 Abort)
    abort_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/intelligence/abort",
      10,
      std::bind(&LandingGearController::abort_callback, this, std::placeholders::_1)
    );

    // Actuator Command to Gazebo Bridge
    cmd_pub_ = this->create_publisher<std_msgs::msg::Float64>(
      "/control/gear_cmd",
      10
    );

    // Status Telemetry
    status_pub_ = this->create_publisher<std_msgs::msg::Float64>(
      "/control/gear_status",
      10
    );

    last_time_ = this->now();
    timer_ = this->create_wall_timer(
      20ms,  // 50 Hz control loop
      std::bind(&LandingGearController::control_loop, this)
    );

    RCLCPP_INFO(
      this->get_logger(),
      "Landing Gear Controller active with Edge AI interlock. Stowed: %.2f rad, Deployed: %.2f rad, Speed: %.2f rad/s",
      stowed_angle_, deployed_angle_, rate_limit_
    );
  }

private:
  void deploy_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    deploy_requested_ = msg->data;
    update_target();
  }

  void abort_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data != abort_active_) {
      abort_active_ = msg->data;
      if (abort_active_) {
        RCLCPP_WARN(
          this->get_logger(),
          "EDGE AI ABORT TRIGGERED: Unsafe touchdown terrain! Inhibiting gear deployment."
        );
      } else {
        RCLCPP_INFO(
          this->get_logger(),
          "Edge AI clearance restored: Terrain SAFE."
        );
      }
      update_target();
    }
  }

  void update_target()
  {
    // Safety Interlock: if terrain is UNSAFE, force STOWED regardless of operator request
    if (abort_active_) {
      target_position_ = stowed_angle_;
    } else {
      target_position_ = deploy_requested_ ? deployed_angle_ : stowed_angle_;
    }
  }

  void control_loop()
  {
    rclcpp::Time now = this->now();
    double dt = (now - last_time_).seconds();
    last_time_ = now;

    if (dt <= 0.0 || dt > 0.1) {
      dt = 0.02;
    }

    // Rate-limited trajectory generation
    double error = target_position_ - current_position_;
    double max_step = rate_limit_ * dt;

    if (std::abs(error) <= max_step) {
      current_position_ = target_position_;
    } else {
      current_position_ += std::copysign(max_step, error);
    }

    // Publish joint position command to bridge
    auto cmd_msg = std_msgs::msg::Float64();
    cmd_msg.data = current_position_;
    cmd_pub_->publish(cmd_msg);

    // Publish telemetry
    auto status_msg = std_msgs::msg::Float64();
    status_msg.data = current_position_;
    status_pub_->publish(status_msg);
  }

  double stowed_angle_;
  double deployed_angle_;
  double rate_limit_;
  double current_position_;
  double target_position_;
  bool deploy_requested_;
  bool abort_active_;

  rclcpp::Time last_time_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr deploy_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr abort_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LandingGearController>());
  rclcpp::shutdown();
  return 0;
}

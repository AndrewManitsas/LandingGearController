// ==============================================================================
// Required Header Files
// ==============================================================================
#include <algorithm>  // Standard library for std::clamp
#include <chrono>     // Time utilities
#include <cmath>      // Math utilities
#include <memory>     // Smart pointers

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"


// ==============================================================================
// Class: StabilatorController
// Subscribes to estimated attitude and publishes elevator commands via PID.
// ==============================================================================
class StabilatorController : public rclcpp::Node
{
public:
  explicit StabilatorController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("stabilator_controller", options),
    pitch_setpoint_(0.0),
    previous_error_(0.0),
    integral_error_(0.0),
    initialized_(false)
  {
    // --------------------------------------------------------------------------
    // PID Parameters (tunable at runtime or via launch file)
    // --------------------------------------------------------------------------
    this->declare_parameter<double>("kp", 1.8);
    this->declare_parameter<double>("ki", 0.05);
    this->declare_parameter<double>("kd", 0.25);
    this->declare_parameter<double>("max_deflection_rad", 0.4363); // 25 deg URDF limit
    this->declare_parameter<double>("anti_windup_limit", 0.20);    // Max integral contribution

    kp_ = this->get_parameter("kp").as_double();
    ki_ = this->get_parameter("ki").as_double();
    kd_ = this->get_parameter("kd").as_double();
    max_deflection_ = this->get_parameter("max_deflection_rad").as_double();
    anti_windup_limit_ = this->get_parameter("anti_windup_limit").as_double();

    // --------------------------------------------------------------------------
    // Subscriptions
    // --------------------------------------------------------------------------
    // 1. Desired pitch angle setpoint (rad)
    setpoint_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      "/control/pitch_setpoint",
      10,
      std::bind(&StabilatorController::setpoint_callback, this, std::placeholders::_1)
    );

    // 2. Feedback attitude from our complementary filter node
    attitude_sub_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
      "/control/attitude",
      10,
      std::bind(&StabilatorController::attitude_callback, this, std::placeholders::_1)
    );

    // --------------------------------------------------------------------------
    // Publisher
    // --------------------------------------------------------------------------
    // Publishes position commands directly forwarded by ros_gz_bridge to the tail joint
    elevator_pub_ = this->create_publisher<std_msgs::msg::Float64>(
      "/control/elevator_cmd",
      10
    );

    RCLCPP_INFO(
      this->get_logger(),
      "Stabilator PID Controller online (Kp: %.2f, Ki: %.2f, Kd: %.2f)",
      kp_, ki_, kd_
    );
  }

private:
  // Updates the target pitch angle setpoint
  void setpoint_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    pitch_setpoint_ = msg->data;
    RCLCPP_INFO(this->get_logger(), "New pitch setpoint received: %.3f rad (%.1f deg)",
      pitch_setpoint_, pitch_setpoint_ * (180.0 / M_PI));
  }

  // Executes the PID control loop synchronously with incoming attitude samples
  void attitude_callback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
  {
    rclcpp::Time current_stamp = msg->header.stamp;
    double current_pitch = msg->vector.y;

    if (!initialized_)
    {
      last_stamp_ = current_stamp;
      previous_error_ = pitch_setpoint_ - current_pitch;
      initialized_ = true;
      return;
    }

    double dt = (current_stamp - last_stamp_).seconds();
    last_stamp_ = current_stamp;

    if (dt <= 0.0 || dt > 0.2)
    {
      dt = 0.01;
    }

    // --------------------------------------------------------------------------
    // Step 1: Compute Control Error
    // --------------------------------------------------------------------------
    double error = pitch_setpoint_ - current_pitch;

    // --------------------------------------------------------------------------
    // Step 2: Proportional Term
    // --------------------------------------------------------------------------
    double p_term = kp_ * error;

    // --------------------------------------------------------------------------
    // Step 3: Integral Term with Anti-Windup Clamping
    // --------------------------------------------------------------------------
    integral_error_ += error * dt;
    integral_error_ = std::clamp(integral_error_, -anti_windup_limit_, anti_windup_limit_);
    double i_term = ki_ * integral_error_;

    // --------------------------------------------------------------------------
    // Step 4: Derivative Term (Derivative on Error)
    // --------------------------------------------------------------------------
    double derivative = (error - previous_error_) / dt;
    double d_term = kd_ * derivative;
    previous_error_ = error;

    // --------------------------------------------------------------------------
    // Step 5: Compute Total Output and Apply Actuator Saturation
    // --------------------------------------------------------------------------
    double command_effort = p_term + i_term + d_term;

    // Clamp command within mechanical limits [-max_deflection, +max_deflection]
    double saturated_cmd = std::clamp(command_effort, -max_deflection_, max_deflection_);

    // --------------------------------------------------------------------------
    // Step 6: Publish Actuator Command
    // --------------------------------------------------------------------------
    auto cmd_msg = std_msgs::msg::Float64();
    cmd_msg.data = saturated_cmd;
    elevator_pub_->publish(cmd_msg);
  }

  // Member variables
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr setpoint_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr attitude_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr elevator_pub_;

  rclcpp::Time last_stamp_;
  double pitch_setpoint_;
  double previous_error_;
  double integral_error_;
  bool initialized_;

  // Tunable gains and limits
  double kp_;
  double ki_;
  double kd_;
  double max_deflection_;
  double anti_windup_limit_;
};


// ==============================================================================
// Main Entry Point
// ==============================================================================
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.append_parameter_override("use_sim_time", true);

  rclcpp::spin(std::make_shared<StabilatorController>(options));
  rclcpp::shutdown();
  return 0;
}
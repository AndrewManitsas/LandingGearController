// ==============================================================================
// Required Header Files
// ==============================================================================
#include <algorithm>  // Provides std::clamp
#include <chrono>     // Time operations
#include <cmath>      // Math functions (std::abs)
#include <memory>     // Smart pointers
#include <string>     // Text handling

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;


// ==============================================================================
// Class: LandingGearController
// Smoothly ramps gear position and reports deployment status.
// ==============================================================================
class LandingGearController : public rclcpp::Node
{
public:
  explicit LandingGearController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("landing_gear_controller", options),
    current_pos_(0.0),
    target_pos_(0.0),
    status_("STOWED")
  {
    // --------------------------------------------------------------------------
    // Parameters (Tunable at runtime)
    // --------------------------------------------------------------------------
    this->declare_parameter<double>("stowed_angle_rad", 0.0);        // 0.0 deg (up inside bay)
    this->declare_parameter<double>("deployed_angle_rad", 1.5708);   // 90.0 deg (down and locked)
    this->declare_parameter<double>("deploy_speed_rad_s", 0.50);     // Takes ~3.1 seconds to swing 90 deg
    this->declare_parameter<double>("loop_rate_hz", 50.0);           // 50 Hz update loop

    stowed_angle_ = this->get_parameter("stowed_angle_rad").as_double();
    deployed_angle_ = this->get_parameter("deployed_angle_rad").as_double();
    speed_rad_s_ = this->get_parameter("deploy_speed_rad_s").as_double();
    double loop_rate = this->get_parameter("loop_rate_hz").as_double();

    // Initialize current position to fully stowed
    current_pos_ = stowed_angle_;
    target_pos_ = stowed_angle_;

    // --------------------------------------------------------------------------
    // Subscriptions
    // --------------------------------------------------------------------------
    // High-level deploy/retract command (true = down, false = up)
    cmd_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/control/gear_deploy",
      10,
      std::bind(&LandingGearController::command_callback, this, std::placeholders::_1)
    );

    // --------------------------------------------------------------------------
    // Publishers
    // --------------------------------------------------------------------------
    // Raw joint command sent to ros_gz_bridge
    gear_pub_ = this->create_publisher<std_msgs::msg::Float64>(
      "/control/gear_cmd",
      10
    );

    // Telemetry status for the supervisory controller
    status_pub_ = this->create_publisher<std_msgs::msg::String>(
      "/control/gear_status",
      10
    );

    // --------------------------------------------------------------------------
    // Control Loop Timer (Fixed 50 Hz)
    // --------------------------------------------------------------------------
    auto period = std::chrono::duration<double>(1.0 / loop_rate);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&LandingGearController::update_loop, this)
    );

    RCLCPP_INFO(
      this->get_logger(),
      "Landing Gear Controller online. Stowed: %.2f rad, Deployed: %.2f rad, Speed: %.2f rad/s",
      stowed_angle_, deployed_angle_, speed_rad_s_
    );
  }

private:
  // ============================================================================
  // Callback: Receives deployment requests
  // ============================================================================
  void command_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data)
    {
      target_pos_ = deployed_angle_;
      RCLCPP_INFO(this->get_logger(), "Deployment command received: EXTENDING gear.");
    }
    else
    {
      target_pos_ = stowed_angle_;
      RCLCPP_INFO(this->get_logger(), "Retraction command received: RETRACTING gear.");
    }
  }

  // ============================================================================
  // Periodic Update Loop (50 Hz)
  // Ramps current_pos_ towards target_pos_ by a maximum step of (speed * dt)
  // ============================================================================
  void update_loop()
  {
    const double dt = 0.02; // 50 Hz = 20 ms
    double max_step = speed_rad_s_ * dt;

    // --------------------------------------------------------------------------
    // Step 1: Ramp Position
    // --------------------------------------------------------------------------
    if (current_pos_ < target_pos_)
    {
      current_pos_ += max_step;
      if (current_pos_ >= target_pos_)
      {
        current_pos_ = target_pos_;
        status_ = "DEPLOYED";
      }
      else
      {
        status_ = "DEPLOYING";
      }
    }
    else if (current_pos_ > target_pos_)
    {
      current_pos_ -= max_step;
      if (current_pos_ <= target_pos_)
      {
        current_pos_ = target_pos_;
        status_ = "STOWED";
      }
      else
      {
        status_ = "RETRACTING";
      }
    }

    // --------------------------------------------------------------------------
    // Step 2: Publish Position Command to Actuator
    // --------------------------------------------------------------------------
    auto cmd_msg = std_msgs::msg::Float64();
    cmd_msg.data = current_pos_;
    gear_pub_->publish(cmd_msg);

    // --------------------------------------------------------------------------
    // Step 3: Publish Status String
    // --------------------------------------------------------------------------
    auto status_msg = std_msgs::msg::String();
    status_msg.data = status_;
    status_pub_->publish(status_msg);
  }

  // Subscriptions, Publishers, and Timers
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr cmd_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr gear_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Internal State
  double current_pos_;
  double target_pos_;
  std::string status_;

  // Configured Parameters
  double stowed_angle_;
  double deployed_angle_;
  double speed_rad_s_;
};


// ==============================================================================
// Main Entry Point
// ==============================================================================
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.append_parameter_override("use_sim_time", true);

  rclcpp::spin(std::make_shared<LandingGearController>(options));
  rclcpp::shutdown();
  return 0;
}
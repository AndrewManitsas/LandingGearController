#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/vector3.hpp"

using namespace std::chrono_literals;

enum class FlightState
{
  APPROACH_DESCENT,
  FLARE,
  TOUCHDOWN,
  GO_AROUND
};

class FlightSupervisor : public rclcpp::Node
{
public:
  FlightSupervisor()
  : Node("flight_supervisor"),
    current_state_(FlightState::APPROACH_DESCENT),
    altitude_agl_(2.0),
    abort_active_(false),
    terrain_verdict_(0),
    pitch_angle_(0.0)
  {
    // Glideslope & flare angle parameters (in radians)
    this->declare_parameter<double>("descent_pitch", -0.045); // -2.6 deg
    this->declare_parameter<double>("flare_pitch", 0.050);    // +2.9 deg
    this->declare_parameter<double>("climb_pitch", 0.200);    // +11.5 deg

    // Decision altitude thresholds (in meters)
    this->declare_parameter<double>("decision_alt", 0.45);
    this->declare_parameter<double>("flare_alt", 0.35);
    this->declare_parameter<double>("touchdown_alt", 0.14);

    descent_pitch_ = this->get_parameter("descent_pitch").as_double();
    flare_pitch_ = this->get_parameter("flare_pitch").as_double();
    climb_pitch_ = this->get_parameter("climb_pitch").as_double();
    decision_alt_ = this->get_parameter("decision_alt").as_double();
    flare_alt_ = this->get_parameter("flare_alt").as_double();
    touchdown_alt_ = this->get_parameter("touchdown_alt").as_double();

    // 1. Ingest filtered ground clearance
    alt_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      "/sensing/altitude_agl",
      rclcpp::SensorDataQoS(),
      [this](const std_msgs::msg::Float64::SharedPtr msg) {
        altitude_agl_ = msg->data;
      }
    );

    // 2. Ingest Edge AI classification verdict & abort flag
    abort_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/intelligence/abort",
      10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        abort_active_ = msg->data;
      }
    );

    verdict_sub_ = this->create_subscription<std_msgs::msg::Int32>(
      "/intelligence/verdict",
      10,
      [this](const std_msgs::msg::Int32::SharedPtr msg) {
        terrain_verdict_ = msg->data;
      }
    );

    // 3. Ingest UAV attitude for monitoring
    attitude_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
      "/control/attitude",
      10,
      [this](const geometry_msgs::msg::Vector3::SharedPtr msg) {
        pitch_angle_ = msg->y;
      }
    );

    // Actuation & Flight Command Publishers
    pitch_setpoint_pub_ = this->create_publisher<std_msgs::msg::Float64>(
      "/control/pitch_setpoint",
      10
    );

    gear_deploy_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      "/control/gear_deploy",
      10
    );

    touchdown_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      "/flight/touchdown",
      10
    );

    state_pub_ = this->create_publisher<std_msgs::msg::String>(
      "/flight/supervisor_state",
      10
    );

    // Supervisory FSM evaluation timer (50 Hz)
    timer_ = this->create_wall_timer(
      20ms,
      std::bind(&FlightSupervisor::evaluate_fsm, this)
    );

    RCLCPP_INFO(this->get_logger(), "Flight Supervisor active. FSM initialized in APPROACH_DESCENT.");
  }

private:
  void evaluate_fsm()
  {
    double commanded_pitch = descent_pitch_;
    bool gear_deploy_cmd = true;
    bool touchdown_cmd = false;
    std::string state_str = "APPROACH_DESCENT";

    switch (current_state_)
    {
      case FlightState::APPROACH_DESCENT:
        state_str = "APPROACH_DESCENT";
        commanded_pitch = descent_pitch_;
        gear_deploy_cmd = true;
        touchdown_cmd = false;

        // Approaching decision gate
        if (altitude_agl_ <= decision_alt_) {
          if (abort_active_) {
            current_state_ = FlightState::GO_AROUND;
            RCLCPP_WARN(
              this->get_logger(),
              "DECISION GATE ALERT: Terrain UNSAFE (verdict=%d, abort=TRUE) at h=%.2fm! WAVE-OFF -> GO-AROUND!",
              terrain_verdict_, altitude_agl_
            );
          } else if (altitude_agl_ <= flare_alt_) {
            current_state_ = FlightState::FLARE;
            RCLCPP_INFO(
              this->get_logger(),
              "DECISION GATE CLEAR: Terrain SAFE. Initiating touchdown FLARE at h=%.2fm.",
              altitude_agl_
            );
          }
        }
        break;

      case FlightState::FLARE:
        state_str = "FLARE";
        commanded_pitch = flare_pitch_;
        gear_deploy_cmd = true;
        touchdown_cmd = false;

        // Emergency abort if terrain becomes unsafe during flare
        if (abort_active_) {
          current_state_ = FlightState::GO_AROUND;
          RCLCPP_WARN(this->get_logger(), "EMERGENCY ABORT DURING FLARE! WAVE-OFF -> GO-AROUND!");
        } else if (altitude_agl_ <= touchdown_alt_) {
          current_state_ = FlightState::TOUCHDOWN;
          RCLCPP_INFO(this->get_logger(), "TOUCHDOWN DETECTED (h=%.2fm). Cutting airspeed.", altitude_agl_);
        }
        break;

      case FlightState::TOUCHDOWN:
        state_str = "TOUCHDOWN";
        commanded_pitch = 0.0;
        gear_deploy_cmd = true;
        touchdown_cmd = true;
        break;

      case FlightState::GO_AROUND:
        state_str = "GO_AROUND";
        commanded_pitch = climb_pitch_;
        gear_deploy_cmd = false; // Retract / hold stowed
        touchdown_cmd = false;
        break;
    }

    // Publish control setpoints
    auto pitch_msg = std_msgs::msg::Float64();
    pitch_msg.data = commanded_pitch;
    pitch_setpoint_pub_->publish(pitch_msg);

    auto gear_msg = std_msgs::msg::Bool();
    gear_msg.data = gear_deploy_cmd;
    gear_deploy_pub_->publish(gear_msg);

    auto td_msg = std_msgs::msg::Bool();
    td_msg.data = touchdown_cmd;
    touchdown_pub_->publish(td_msg);

    auto state_msg = std_msgs::msg::String();
    state_msg.data = state_str;
    state_pub_->publish(state_msg);
  }

  FlightState current_state_;
  double altitude_agl_;
  bool abort_active_;
  int32_t terrain_verdict_;
  double pitch_angle_;

  double descent_pitch_;
  double flare_pitch_;
  double climb_pitch_;
  double decision_alt_;
  double flare_alt_;
  double touchdown_alt_;

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr alt_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr abort_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr verdict_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr attitude_sub_;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pitch_setpoint_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gear_deploy_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr touchdown_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FlightSupervisor>());
  rclcpp::shutdown();
  return 0;
}

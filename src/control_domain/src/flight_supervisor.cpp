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
  ARMING,
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
	current_state_(FlightState::ARMING),
	altitude_agl_(-1.0),
	abort_active_(false),
	terrain_verdict_(0),
	pitch_angle_(0.0),
	arm_time_sec_(0.0)
  {
	this->declare_parameter<double>("descent_pitch", -0.045); // -2.6 deg
	this->declare_parameter<double>("flare_pitch", 0.050);    // +2.9 deg
	this->declare_parameter<double>("climb_pitch", 0.200);    // +11.5 deg
	this->declare_parameter<double>("decision_alt", 0.90);    // Begin deploy check
	this->declare_parameter<double>("flare_alt", 0.35);
	this->declare_parameter<double>("touchdown_alt", 0.14);

	descent_pitch_ = this->get_parameter("descent_pitch").as_double();
	flare_pitch_ = this->get_parameter("flare_pitch").as_double();
	climb_pitch_ = this->get_parameter("climb_pitch").as_double();
	decision_alt_ = this->get_parameter("decision_alt").as_double();
	flare_alt_ = this->get_parameter("flare_alt").as_double();
	touchdown_alt_ = this->get_parameter("touchdown_alt").as_double();

	alt_sub_ = this->create_subscription<std_msgs::msg::Float64>(
	  "/sensing/altitude_agl",
	  rclcpp::SensorDataQoS(),
	  [this](const std_msgs::msg::Float64::SharedPtr msg) {
		altitude_agl_ = msg->data;
	  }
	);

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

	attitude_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
	  "/control/attitude",
	  10,
	  [this](const geometry_msgs::msg::Vector3::SharedPtr msg) {
		pitch_angle_ = msg->y;
	  }
	);

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

	timer_ = this->create_wall_timer(
	  20ms,
	  std::bind(&FlightSupervisor::evaluate_fsm, this)
	);

	RCLCPP_INFO(this->get_logger(), "Flight Supervisor online in ARMING state. Waiting for spawn...");
  }

private:
  void evaluate_fsm()
  {
	arm_time_sec_ += 0.02;

	double commanded_pitch = 0.0;
	bool gear_deploy_cmd = false; // Default: GEAR STOWED
	bool touchdown_cmd = false;
	std::string state_str = "ARMING";

	switch (current_state_)
	{
	  case FlightState::ARMING:
		state_str = "ARMING";
		commanded_pitch = 0.0;
		gear_deploy_cmd = false; // Hold gear stowed
		touchdown_cmd = false;

		// Allow 5.0 seconds for Gazebo spawn and wait for valid altitude > 0.4m
		if (arm_time_sec_ > 5.0 && altitude_agl_ > 0.40) {
		  current_state_ = FlightState::APPROACH_DESCENT;
		  RCLCPP_INFO(
			this->get_logger(),
			"SYSTEM ARMED (h=%.2fm). Engaging forward glideslope descent. Gear STOWED.",
			altitude_agl_
		  );
		}
		break;

	  case FlightState::APPROACH_DESCENT:
		state_str = "APPROACH_DESCENT";
		commanded_pitch = descent_pitch_;
		touchdown_cmd = false;

		// Deployment rule: only deploy gear if low enough AND terrain is clear of aborts
		if (altitude_agl_ <= decision_alt_ && !abort_active_) {
		  gear_deploy_cmd = true; // Clear runway sector: Deploy gear
		} else {
		  gear_deploy_cmd = false; // Over obstacles or high altitude: Keep stowed
		}

		// Decision logic
		if (altitude_agl_ <= decision_alt_) {
		  if (abort_active_ && altitude_agl_ <= 0.60) {
			current_state_ = FlightState::GO_AROUND;
			RCLCPP_WARN(
			  this->get_logger(),
			  "DECISION ALERT: Unsafe terrain (verdict=%d) at h=%.2fm! WAVE-OFF -> GO-AROUND!",
			  terrain_verdict_, altitude_agl_
			);
		  } else if (!abort_active_ && altitude_agl_ <= flare_alt_) {
			current_state_ = FlightState::FLARE;
			RCLCPP_INFO(this->get_logger(), "DECISION CLEAR: Terrain SAFE. Initiating FLARE at h=%.2fm.", altitude_agl_);
		  }
		}
		break;

	  case FlightState::FLARE:
		state_str = "FLARE";
		commanded_pitch = flare_pitch_;
		gear_deploy_cmd = true;
		touchdown_cmd = false;

		if (abort_active_) {
		  current_state_ = FlightState::GO_AROUND;
		  RCLCPP_WARN(this->get_logger(), "EMERGENCY ABORT DURING FLARE! WAVE-OFF -> GO-AROUND!");
		} else if (altitude_agl_ <= touchdown_alt_) {
		  current_state_ = FlightState::TOUCHDOWN;
		  RCLCPP_INFO(this->get_logger(), "TOUCHDOWN CONFIRMED (h=%.2fm). Halting airspeed.", altitude_agl_);
		}
		break;

	  case FlightState::TOUCHDOWN:
		state_str = "TOUCHDOWN";
		commanded_pitch = 0.0;
		gear_deploy_cmd = true;
		touchdown_cmd = true; // Cut forward propulsion
		break;

	  case FlightState::GO_AROUND:
		state_str = "GO_AROUND";
		commanded_pitch = climb_pitch_;
		gear_deploy_cmd = false; // Retract / hold stowed
		touchdown_cmd = false;
		break;
	}

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
  double arm_time_sec_;

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
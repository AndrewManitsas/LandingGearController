#include <chrono>
#include <cmath>
#include <memory>
#include <vector>
#include <algorithm>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

using namespace std::chrono_literals;

class SensorProcessor : public rclcpp::Node
{
public:
	SensorProcessor()
	: Node("sensor_processor"),
		sonar_alt_(0.85),
		tof_fore_(0.85),
		tof_port_(0.85),
		tof_starboard_(0.85),
		sonar_var_(0.0001),
		tof_var_(0.0001)
	{
		auto sensor_qos = rclcpp::SensorDataQoS();

		sonar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
			"/sensing/sonar/scan",
			sensor_qos,
			std::bind(&SensorProcessor::sonar_callback, this, std::placeholders::_1)
		);

		tof_fore_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
			"/sensing/tof/fore",
			sensor_qos,
			[this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
				tof_fore_ = extract_min_valid_range(msg);
				update_tof_variance();
			}
		);

		tof_port_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
			"/sensing/tof/port",
			sensor_qos,
			[this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
				tof_port_ = extract_min_valid_range(msg);
				update_tof_variance();
			}
		);

		tof_starboard_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
			"/sensing/tof/starboard",
			sensor_qos,
			[this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
				tof_starboard_ = extract_min_valid_range(msg);
				update_tof_variance();
			}
		);

		alt_pub_ = this->create_publisher<std_msgs::msg::Float64>("/sensing/altitude_agl", 10);
		features_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/sensing/features", 10);

		timer_ = this->create_wall_timer(20ms, std::bind(&SensorProcessor::publish_telemetry, this));
		RCLCPP_INFO(this->get_logger(), "Sensor Processor online. Feature mapping aligned with model_rules.hpp.");
	}

private:
	double extract_min_valid_range(const sensor_msgs::msg::LaserScan::SharedPtr & msg)
	{
		if (msg->ranges.empty()) return 0.85;

		double min_val = std::numeric_limits<double>::infinity();
		for (float r : msg->ranges) {
			if (std::isfinite(r) && r >= msg->range_min && r <= msg->range_max) {
				if (r < min_val) {
					min_val = r;
				}
			}
		}
		return std::isfinite(min_val) ? min_val : 0.85;
	}

	void update_tof_variance()
	{
		double mean = (tof_fore_ + tof_port_ + tof_starboard_) / 3.0;
		double var = ((tof_fore_ - mean) * (tof_fore_ - mean) +
									(tof_port_ - mean) * (tof_port_ - mean) +
									(tof_starboard_ - mean) * (tof_starboard_ - mean)) / 3.0;
		tof_var_ = std::max(0.00001, var);
	}

	void sonar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
	{
		std::vector<double> valid_ranges;
		for (float r : msg->ranges) {
			if (std::isfinite(r) && r >= msg->range_min && r <= msg->range_max) {
				valid_ranges.push_back(r);
			}
		}

		if (valid_ranges.empty()) return;

		double sum = 0.0;
		for (double r : valid_ranges) sum += r;
		double mean = sum / valid_ranges.size();
		sonar_alt_ = mean;

		double var_sum = 0.0;
		for (double r : valid_ranges) {
			var_sum += (r - mean) * (r - mean);
		}
		sonar_var_ = std::max(0.00001, var_sum / valid_ranges.size());
	}

	void publish_telemetry()
	{
		double central_alt = (tof_fore_ + tof_port_ + tof_starboard_) / 3.0;

		auto alt_msg = std_msgs::msg::Float64();
		alt_msg.data = central_alt;
		alt_pub_->publish(alt_msg);

		// Delta d: Acoustic - Optical
		double delta_d = sonar_alt_ - central_alt;

		// Pitch & Roll Slope Triangulation
		double port_star_mid = (tof_port_ + tof_starboard_) * 0.5;
		double pitch_slope = std::atan2(tof_fore_ - port_star_mid, 0.070);
		double roll_slope = std::atan2(tof_port_ - tof_starboard_, 0.078);

		// Total slope magnitude (always non-negative)
		double total_slope = std::sqrt(pitch_slope * pitch_slope + roll_slope * roll_slope);

		// Feature mapping strictly matching model_rules.hpp:
		// [0] mean_tof
		// [1] spatial_variance (tof height variance)
		// [2] delta_d
		// [3] delta_d_variance
		// [4] beta_slope
		// [5] omega_norm
		// [6] tilt_error (non-negative inclination magnitude for <= 0.137855 threshold)
		auto feat_msg = std_msgs::msg::Float32MultiArray();
		feat_msg.data.resize(7);
		feat_msg.data[0] = static_cast<float>(central_alt);
		feat_msg.data[1] = static_cast<float>(tof_var_);
		feat_msg.data[2] = static_cast<float>(delta_d);
		feat_msg.data[3] = static_cast<float>(sonar_var_);
		feat_msg.data[4] = static_cast<float>(total_slope);
		feat_msg.data[5] = static_cast<float>(std::abs(roll_slope));
		feat_msg.data[6] = static_cast<float>(total_slope);

		features_pub_->publish(feat_msg);
	}

	double sonar_alt_;
	double tof_fore_;
	double tof_port_;
	double tof_starboard_;
	double sonar_var_;
	double tof_var_;

	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sonar_sub_;
	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr tof_fore_sub_;
	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr tof_port_sub_;
	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr tof_starboard_sub_;

	rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr alt_pub_;
	rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr features_pub_;
	rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<SensorProcessor>());
	rclcpp::shutdown();
	return 0;
}
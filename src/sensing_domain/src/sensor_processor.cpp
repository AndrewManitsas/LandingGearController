// ==============================================================================
// Required Header Files
// ==============================================================================
#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <memory>
#include <numeric>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"

using namespace std::chrono_literals;

// ==============================================================================
// Helper Function: Moving Average & Variance
// ==============================================================================
struct BufferStats
{
  double mean = 0.0;
  double variance = 0.0;
};

BufferStats compute_stats(const std::deque<double> & buffer)
{
  BufferStats stats;
  if (buffer.empty())
  {
    return stats;
  }

  double sum = std::accumulate(buffer.begin(), buffer.end(), 0.0);
  stats.mean = sum / static_cast<double>(buffer.size());

  double sq_sum = 0.0;
  for (double val : buffer)
  {
    sq_sum += (val - stats.mean) * (val - stats.mean);
  }
  stats.variance = sq_sum / static_cast<double>(buffer.size());
  return stats;
}


// ==============================================================================
// Class: SensorProcessor
// ==============================================================================
class SensorProcessor : public rclcpp::Node
{
public:
  explicit SensorProcessor(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("sensor_processor", options),
    raw_tof_fore_(0.0),
    raw_tof_port_(0.0),
    raw_tof_starboard_(0.0),
    raw_sonar_(0.0),
    omega_norm_(0.0),
    current_pitch_(0.0),
    has_tof_fore_(false),
    has_tof_port_(false),
    has_tof_starboard_(false),
    has_sonar_(false)
  {
    // --------------------------------------------------------------------------
    // Parameters
    // --------------------------------------------------------------------------
    this->declare_parameter<int>("window_size", 10);
    this->declare_parameter<double>("array_radius_m", 0.06);     // 6 cm equilateral array radius
    this->declare_parameter<double>("min_valid_range_m", 0.03);  // 3 cm min range
    this->declare_parameter<double>("max_valid_range_m", 4.00);  // 4 m max range

    window_size_ = this->get_parameter("window_size").as_int();
    array_radius_ = this->get_parameter("array_radius_m").as_double();
    min_range_ = this->get_parameter("min_valid_range_m").as_double();
    max_range_ = this->get_parameter("max_valid_range_m").as_double();

    // --------------------------------------------------------------------------
    // Subscriptions: Range Sensors (SensorDataQoS matching Gazebo bridge)
    // --------------------------------------------------------------------------
    tof_fore_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/sensing/tof/fore",
      rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::LaserScan::SharedPtr msg)
      {
        raw_tof_fore_ = extract_scan_range(msg);
        has_tof_fore_ = true;
      }
    );

    tof_port_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/sensing/tof/port",
      rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::LaserScan::SharedPtr msg)
      {
        raw_tof_port_ = extract_scan_range(msg);
        has_tof_port_ = true;
      }
    );

    tof_starboard_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/sensing/tof/starboard",
      rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::LaserScan::SharedPtr msg)
      {
        raw_tof_starboard_ = extract_scan_range(msg);
        has_tof_starboard_ = true;
      }
    );

    sonar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/sensing/sonar/scan",
      rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::LaserScan::SharedPtr msg)
      {
        raw_sonar_ = extract_scan_range(msg);
        has_sonar_ = true;
      }
    );

    // Angular rate for feature vector turbulence metric
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/sensing/imu/data_raw",
      rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Imu::SharedPtr msg)
      {
        double wx = msg->angular_velocity.x;
        double wy = msg->angular_velocity.y;
        double wz = msg->angular_velocity.z;
        omega_norm_ = std::sqrt(wx * wx + wy * wy + wz * wz);
      }
    );

    // Current pitch for tilt-to-terrain alignment metric
    attitude_sub_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
      "/control/attitude",
      10,
      [this](const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
      {
        current_pitch_ = msg->vector.y;
      }
    );

    // --------------------------------------------------------------------------
    // Publishers
    // --------------------------------------------------------------------------
    // 1. Primary ground clearance (h_AGL) for control and landing gear deployment
    altitude_pub_ = this->create_publisher<std_msgs::msg::Float64>(
      "/sensing/altitude_agl",
      10
    );

    // 2. Synchronous 7D feature vector for Intelligence Domain (Phase 4)
    features_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
      "/sensing/features",
      10
    );

    // --------------------------------------------------------------------------
    // 40 Hz Processing & Publishing Timer
    // --------------------------------------------------------------------------
    timer_ = this->create_wall_timer(
      25ms,
      std::bind(&SensorProcessor::process_and_publish, this)
    );

    RCLCPP_INFO(this->get_logger(), "SensorProcessor initialized (Window: %d samples, Array radius: %.2f m)",
      window_size_, array_radius_);
  }

private:
  // Helper to extract and validate range from LaserScan message
  double extract_scan_range(const sensor_msgs::msg::LaserScan::SharedPtr & msg)
  {
    if (msg->ranges.empty())
    {
      return max_range_;
    }
    double r = msg->ranges[0];
    if (!std::isfinite(r) || r < min_range_)
    {
      return min_range_;
    }
    if (r > max_range_)
    {
      return max_range_;
    }
    return r;
  }

  // Periodic pipeline executing at 40 Hz
  void process_and_publish()
  {
    // Wait until all four ventral rangefinders have delivered at least one packet
    if (!has_tof_fore_ || !has_tof_port_ || !has_tof_starboard_ || !has_sonar_)
    {
      return;
    }

    // --------------------------------------------------------------------------
    // 1. Update Rolling Temporal Buffers
    // --------------------------------------------------------------------------
    push_to_buffer(tof_fore_buf_, raw_tof_fore_);
    push_to_buffer(tof_port_buf_, raw_tof_port_);
    push_to_buffer(tof_starboard_buf_, raw_tof_starboard_);
    push_to_buffer(sonar_buf_, raw_sonar_);

    double d1 = compute_stats(tof_fore_buf_).mean;
    double d2 = compute_stats(tof_port_buf_).mean;
    double d3 = compute_stats(tof_starboard_buf_).mean;
    double d_sonar = compute_stats(sonar_buf_).mean;

    // --------------------------------------------------------------------------
    // 2. Compute Mean ToF Altitude (h_AGL) and Spatial Dispersion
    // --------------------------------------------------------------------------
    double mean_tof = (d1 + d2 + d3) / 3.0;
    double spatial_variance = ((d1 - mean_tof) * (d1 - mean_tof) +
                               (d2 - mean_tof) * (d2 - mean_tof) +
                               (d3 - mean_tof) * (d3 - mean_tof)) / 3.0;

    // --------------------------------------------------------------------------
    // 3. Differential Texture Signature: delta_d = Sonar - mean(ToF)
    // --------------------------------------------------------------------------
    double delta_d = d_sonar - mean_tof;
    push_to_buffer(delta_d_buf_, delta_d);
    double delta_d_variance = compute_stats(delta_d_buf_).variance;

    // --------------------------------------------------------------------------
    // 4. Terrain Normal Vector and Slope (beta) Extraction
    // Array layout: P1 = (r, 0, d1), P2 = (-r/2, r*sqrt(3)/2, d2), P3 = (-r/2, -r*sqrt(3)/2, d3)
    // Surface normal N = (P2 - P1) x (P3 - P1)
    // --------------------------------------------------------------------------
    double r = array_radius_;
    double nx = (std::sqrt(3.0) / 2.0) * r * (d3 + d2 - 2.0 * d1);
    double ny = 1.5 * r * (d3 - d2);
    double nz = 1.5 * std::sqrt(3.0) * r * r;

    double norm_N = std::sqrt(nx * nx + ny * ny + nz * nz);
    double beta = 0.0;
    if (norm_N > 1e-6)
    {
      double cos_beta = std::clamp(nz / norm_N, -1.0, 1.0);
      beta = std::acos(cos_beta); // Slope angle in radians relative to gravity vector
    }

    double tilt_error = std::abs(current_pitch_ - beta);

    // --------------------------------------------------------------------------
    // 5. Publish Clean Altitude Above Ground Level
    // --------------------------------------------------------------------------
    auto alt_msg = std_msgs::msg::Float64();
    alt_msg.data = mean_tof;
    altitude_pub_->publish(alt_msg);

    // --------------------------------------------------------------------------
    // 6. Publish 7D Feature Vector for Edge AI (Intelligence Domain)
    // --------------------------------------------------------------------------
    auto feat_msg = std_msgs::msg::Float32MultiArray();
    feat_msg.data = {
      static_cast<float>(mean_tof),          // 1. Mean ToF altitude
      static_cast<float>(spatial_variance),  // 2. Spatial dispersion across 3 ToFs
      static_cast<float>(delta_d),           // 3. Acoustic-optical differential
      static_cast<float>(delta_d_variance),  // 4. Temporal variance of differential
      static_cast<float>(beta),              // 5. Ground slope angle (rad)
      static_cast<float>(omega_norm_),       // 6. Angular velocity norm (turbulence)
      static_cast<float>(tilt_error)         // 7. Tilt error (rad)
    };
    features_pub_->publish(feat_msg);
  }

  void push_to_buffer(std::deque<double> & buffer, double value)
  {
    buffer.push_back(value);
    if (buffer.size() > static_cast<size_t>(window_size_))
    {
      buffer.pop_front();
    }
  }

  // Parameters
  int window_size_;
  double array_radius_;
  double min_range_;
  double max_range_;

  // Buffers
  std::deque<double> tof_fore_buf_;
  std::deque<double> tof_port_buf_;
  std::deque<double> tof_starboard_buf_;
  std::deque<double> sonar_buf_;
  std::deque<double> delta_d_buf_;

  // Latest raw measurements
  double raw_tof_fore_;
  double raw_tof_port_;
  double raw_tof_starboard_;
  double raw_sonar_;
  double omega_norm_;
  double current_pitch_;

  bool has_tof_fore_;
  bool has_tof_port_;
  bool has_tof_starboard_;
  bool has_sonar_;

  // Subscriptions & Publishers
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr tof_fore_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr tof_port_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr tof_starboard_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sonar_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr attitude_sub_;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr altitude_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr features_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};


// ==============================================================================
// Main Entry Point
// ==============================================================================
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.append_parameter_override("use_sim_time", true);

  rclcpp::spin(std::make_shared<SensorProcessor>(options));
  rclcpp::shutdown();
  return 0;
}
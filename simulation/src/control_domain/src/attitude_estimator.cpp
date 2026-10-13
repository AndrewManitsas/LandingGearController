// ==============================================================================
// Required Header Files (Libraries)
// ==============================================================================
#include <chrono>   // Standard C++ library for handling time and clocks
#include <cmath>    // Standard C++ math library (provides atan2, sqrt)
#include <memory>   // Standard C++ library for smart pointers (std::make_shared)

#include "rclcpp/rclcpp.hpp"                           // Core ROS 2 C++ client library
#include "sensor_msgs/msg/imu.hpp"                     // ROS 2 message definition for raw IMU data
#include "geometry_msgs/msg/vector3_stamped.hpp"       // ROS 2 message definition for 3D vector with a timestamp


// ==============================================================================
// Class: AttitudeEstimator
// Inherits from "rclcpp::Node" to become an active ROS 2 node.
// ==============================================================================
class AttitudeEstimator : public rclcpp::Node
{
public:
  // Constructor: Executes once when the node is created in memory
  explicit AttitudeEstimator(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("attitude_estimator", options), initialized_(false), roll_(0.0), pitch_(0.0)
  {
    // Declare a ROS parameter named 'alpha' with a default value of 0.90.
    // This allows alpha to be changed via a launch file or terminal without recompiling.
    this->declare_parameter<double>("alpha", 0.90);
    alpha_ = this->get_parameter("alpha").as_double();

    // --------------------------------------------------------------------------
    // Subscriber Setup
    // --------------------------------------------------------------------------
    // Subscribe to the raw IMU topic coming from Gazebo / ros_gz_bridge.
    //
    // Note on SensorDataQoS():
    // Sensor streams typically use 'Best Effort' delivery (drop lost packets rather
    // than retrying). Using SensorDataQoS() ensures our node's subscriber profile
    // matches the publisher profile used by ros_gz_bridge.
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/sensing/imu/data_raw",
      rclcpp::SensorDataQoS(),
      std::bind(&AttitudeEstimator::imu_callback, this, std::placeholders::_1)
    );

    // --------------------------------------------------------------------------
    // Publisher Setup
    // --------------------------------------------------------------------------
    // Create a publisher that outputs our estimated attitude angles (roll, pitch)
    // with a queue depth of 10 messages.
    attitude_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
      "/control/attitude",
      10
    );

    RCLCPP_INFO(this->get_logger(), "Attitude Estimator initialized (alpha: %.2f)", alpha_);
  }

private:
  // ============================================================================
  // Callback Function: Executes automatically whenever a new IMU message arrives
  // ============================================================================
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    // Extract the simulation timestamp from the incoming message header
    rclcpp::Time current_stamp = msg->header.stamp;

    // --------------------------------------------------------------------------
    // Step 1: Extract Linear Accelerations (in m/s^2)
    // --------------------------------------------------------------------------
    double ax = msg->linear_acceleration.x;
    double ay = msg->linear_acceleration.y;
    double az = msg->linear_acceleration.z;

    // --------------------------------------------------------------------------
    // Step 2: Extract Angular Velocities / Gyroscope Rates (in rad/s)
    // --------------------------------------------------------------------------
    double wx = msg->angular_velocity.x;  // Roll rate  (rotation around X)
    double wy = msg->angular_velocity.y;  // Pitch rate (rotation around Y)

    // --------------------------------------------------------------------------
    // Step 3: Compute Tilt Angles from Accelerometer
    // When stationary or moving smoothly, the accelerometer measures the direction
    // of Earth's gravity vector (approx. 9.81 m/s^2 pointing downwards).
    //
    // Roll  (phi)   = atan2(ay, az)
    // Pitch (theta) = atan2(-ax, sqrt(ay^2 + az^2))
    // --------------------------------------------------------------------------
    double roll_acc = std::atan2(ay, az);
    double pitch_acc = std::atan2(-ax, std::sqrt(ay * ay + az * az));

    // --------------------------------------------------------------------------
    // Step 4: First-Run Initialization
    // On the very first reading, initialize the filter directly to the accelerometer
    // values so we don't start from 0.0 rad if the UAV begins at a tilted angle.
    // --------------------------------------------------------------------------
    if (!initialized_)
    {
      last_stamp_ = current_stamp;
      roll_ = roll_acc;
      pitch_ = pitch_acc;
      initialized_ = true;

      RCLCPP_INFO(this->get_logger(), "First IMU sample received. Complementary filter active.");
      return;
    }

    // --------------------------------------------------------------------------
    // Step 5: Compute Delta Time (dt)
    // Time elapsed between the previous message and this message (in seconds).
    // --------------------------------------------------------------------------
    double dt = (current_stamp - last_stamp_).seconds();
    last_stamp_ = current_stamp;

    // Safety guard: If simulation time paused, jumped backward, or lagged heavily,
    // default dt to 10 ms (0.01s, matching the expected 100 Hz rate) to prevent spikes.
    if (dt <= 0.0 || dt > 0.2)
    {
      dt = 0.01;
    }

    // --------------------------------------------------------------------------
    // Step 6: Complementary Filter Fusion
    //
    // Gyroscopes are responsive and smooth in the short term, but drift over time.
    // Accelerometers are noisy in the short term, but have zero long-term drift.
    //
    // Complementary filter formula:
    // Angle = alpha * (Previous_Angle + Gyro_Rate * dt) + (1 - alpha) * Accel_Angle
    //
    // With alpha = 0.90:
    //  - 90% of the estimate relies on high-speed gyro integration
    //  - 10% corrects drift using the gravity vector
    // --------------------------------------------------------------------------
    roll_ = alpha_ * (roll_ + wx * dt) + (1.0 - alpha_) * roll_acc;
    pitch_ = alpha_ * (pitch_ + wy * dt) + (1.0 - alpha_) * pitch_acc;

    // --------------------------------------------------------------------------
    // Step 7: Construct and Publish Output Message
    // --------------------------------------------------------------------------
    auto att_msg = geometry_msgs::msg::Vector3Stamped();

    // Preserve the original sensor timestamp and coordinate frame
    att_msg.header.stamp = current_stamp;
    att_msg.header.frame_id = "base_link";

    // Populate Euler angles in radians:
    att_msg.vector.x = roll_;   // Roll angle  (rad)
    att_msg.vector.y = pitch_;  // Pitch angle (rad)
    att_msg.vector.z = 0.0;     // Yaw angle   (IMU alone cannot observe yaw without a magnetometer)

    // Send the message over ROS 2
    attitude_pub_->publish(att_msg);
  }

  // Member variables (state retained across callbacks)
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr attitude_pub_;

  rclcpp::Time last_stamp_;  // Timestamp of the previous IMU sample
  bool initialized_;         // True once the first reading has been processed
  double alpha_;             // Filter weighting factor (e.g. 0.90)
  double roll_;              // Current estimated roll angle (radians)
  double pitch_;             // Current estimated pitch angle (radians)
};


// ==============================================================================
// Main Entry Point
// ==============================================================================
int main(int argc, char **argv)
{
  // Initialize the ROS 2 communications middleware
  rclcpp::init(argc, argv);

  // Configure NodeOptions to default use_sim_time to true.
  // This tells the node to read clock ticks from the Gazebo /clock topic
  // rather than the computer's physical system clock.
  rclcpp::NodeOptions options;
  options.append_parameter_override("use_sim_time", true);

  // Instantiate the node and keep it running (spinning) to process callbacks
  rclcpp::spin(std::make_shared<AttitudeEstimator>(options));

  // Clean shutdown when the node is terminated (e.g., via Ctrl+C)
  rclcpp::shutdown();
  return 0;
}
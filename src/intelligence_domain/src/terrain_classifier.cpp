#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/bool.hpp"

#include "intelligence_domain/model_rules.hpp"

class TerrainClassifier : public rclcpp::Node
{
public:
  explicit TerrainClassifier(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("terrain_classifier", options)
  {
    features_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/sensing/features",
      rclcpp::SensorDataQoS(),
      std::bind(&TerrainClassifier::features_callback, this, std::placeholders::_1)
    );

    verdict_pub_ = this->create_publisher<std_msgs::msg::Int32>(
      "/intelligence/verdict",
      10
    );

    abort_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      "/intelligence/abort",
      10
    );

    RCLCPP_INFO(this->get_logger(), "TerrainClassifier initialized. Edge AI inference active.");
  }

private:
  void features_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 7)
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "Received feature vector with invalid size: %zu (expected >= 7)",
        msg->data.size()
      );
      return;
    }

    // Run deterministic microsecond C++ inference
    int32_t prediction = intelligence_domain::predict_terrain(msg->data.data());

    // 0: SAFE, 1: UNSAFE_SLOPE, 2: UNSAFE_VEGETATION
    bool abort_required = (prediction != static_cast<int32_t>(intelligence_domain::TerrainVerdict::SAFE));

    auto verdict_msg = std_msgs::msg::Int32();
    verdict_msg.data = prediction;
    verdict_pub_->publish(verdict_msg);

    auto abort_msg = std_msgs::msg::Bool();
    abort_msg.data = abort_required;
    abort_pub_->publish(abort_msg);
  }

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr features_sub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr verdict_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr abort_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.append_parameter_override("use_sim_time", true);

  rclcpp::spin(std::make_shared<TerrainClassifier>(options));
  rclcpp::shutdown();
  return 0;
}

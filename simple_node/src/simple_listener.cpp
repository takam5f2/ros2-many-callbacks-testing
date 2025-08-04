#include "simple_node/simple_listener.hpp"
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include "yaml-cpp/yaml.h"

namespace simple_node
{
SimpleListener::SimpleListener(const std::string &default_node_name, const YAML::Node &config,
                               const rclcpp::NodeOptions &options)
  : Node(default_node_name, options)
{
  std::string node_name = this->get_name();

  RCLCPP_INFO(this->get_logger(), "%s node has been created.", node_name.c_str());

  init(config);
}

SimpleListener::~SimpleListener()
{
  RCLCPP_INFO(this->get_logger(), "SimpleListener node is being destroyed.");
}

int SimpleListener::init(const YAML::Node &config)
{
  if (subscribers_.size() > 0) {
    return -1;
  }

  std::string node_name = this->get_name();
  if (!config["callbacks"] || !config["callbacks"].IsSequence()) {
    RCLCPP_ERROR(this->get_logger(), "No 'callbacks' sequence found for node '%s'", node_name.c_str());
    return -1;
  }

  unsigned int callback_idx = 0;
  for (const auto& cb_config : config["callbacks"]) {
    if (!cb_config["topic"] || !cb_config["topic"].IsScalar()) {
      RCLCPP_ERROR(this->get_logger(), "No 'topic' found in callback configuration for node '%s'", node_name.c_str());
      continue;
    }
    auto topic = cb_config["topic"].as<std::string>();
    auto silent = false; // Default to not silent
    if (config["silent"] && config["silent"].IsScalar()) {
      silent = config["silent"].as<bool>();
    }

    receiving_counters_.push_back(static_cast<unsigned int>(0));
    auto subscriber = this->create_subscription<std_msgs::msg::String>(
      topic, 10,
      [this, topic, callback_idx, silent](const std_msgs::msg::String::SharedPtr msg) {
      if (!silent) {
        RCLCPP_INFO(this->get_logger(), "%d Received message on '%s': %s", receiving_counters_[callback_idx], topic.c_str(), msg->data.c_str());
      }
      receiving_counters_[callback_idx]++;
      }
    );
    subscribers_.push_back(subscriber);
    callback_idx++;
  }
  return 0;
}  

} // namespace simple_node

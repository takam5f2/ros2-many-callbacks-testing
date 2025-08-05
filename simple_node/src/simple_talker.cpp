#include "simple_node/simple_talker.hpp"
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include "yaml-cpp/yaml.h"

namespace simple_node
{

SimpleTalker::SimpleTalker(const std::string &default_node_name, const YAML::Node &config,
                           const rclcpp::NodeOptions &options)
  : Node(default_node_name, options)
{
  std::string node_name = this->get_name();

  RCLCPP_INFO(this->get_logger(), "%s node has been created.", node_name.c_str());

  if (config["callbacks"] && config["callbacks"].IsSequence()) {
    init(config);
  }
}

SimpleTalker::~SimpleTalker()
{
  RCLCPP_INFO(this->get_logger(), "SimpleTalker node is being destroyed.");
}

int SimpleTalker::init(const YAML::Node &config)
{
  if (publishers_.size() > 0 || timers_.size() > 0) {
    return -1; // Already initialized
  }

  std::string node_name = this->get_name();

  if (!config["callbacks"] || !config["callbacks"].IsSequence()) {
    RCLCPP_ERROR(this->get_logger(), "No 'callbacks' sequence found for node '%s'", node_name.c_str());
    return -1;
  }

  unsigned int callback_idx = 0;
  for (const auto &cb_config : config["callbacks"])
  {
    auto topic = cb_config["topic"].as<std::string>();
    auto frequency = cb_config["frequency"].as<double>();
    auto message_text = cb_config["message"].as<std::string>();
    auto launch = true; // Default to not silent
    if (cb_config["launch"] && cb_config["launch"].IsScalar()) {
      launch = cb_config["launch"].as<bool>();
    }    

    if (launch) {
      auto publisher = this->create_publisher<std_msgs::msg::String>(topic, 10);
      publishers_.push_back(publisher);
      publishing_counters_.push_back(static_cast<unsigned int>(0));
    
      auto timer_callback = [this, publisher, message_text, callback_idx]() -> void {
        auto message = std_msgs::msg::String();
        message.data = message_text + " " + std::to_string(publishing_counters_[callback_idx]++);
        publisher->publish(message);
      };

      auto timer = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / frequency),
        timer_callback
      );
      timers_.push_back(timer);

      RCLCPP_INFO(this->get_logger(), "Publishing to '%s' at %f Hz", topic.c_str(), frequency);

    } else {
      publishing_counters_.push_back(static_cast<unsigned int>(0));

      auto timer_callback = [this, topic, message_text, callback_idx]() -> void
      {
        publishing_counters_[callback_idx]++;
      };

      auto timer = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / frequency),
        timer_callback
      );
      timers_.push_back(timer);

      RCLCPP_INFO(this->get_logger(), "Timer for '%s' set at %f Hz but not publishing messages", topic.c_str(), frequency);
    }

    callback_idx++;
  }

  return 0;
}
}

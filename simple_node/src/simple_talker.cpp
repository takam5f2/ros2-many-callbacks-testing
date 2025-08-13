#include "simple_node/simple_talker.hpp"
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <std_msgs/msg/string.hpp>
#include <random>
#include <string>
#include <vector>

#include "yaml-cpp/yaml.h"
#include "simple_node/message_generator.hpp"

namespace simple_node
{

SimpleTalker::SimpleTalker(const std::string &default_node_name, const YAML::Node &config, unsigned int random_seed,
                           const rclcpp::NodeOptions &options)
  : Node(default_node_name, options)
{
  std::string node_name = this->get_name();

  RCLCPP_INFO(this->get_logger(), "%s node has been created.", node_name.c_str());

  if (options.use_intra_process_comms()) {
    RCLCPP_INFO(this->get_logger(), "Intra-process communication is enabled for node '%s'.", node_name.c_str());
  }

  if (config["callbacks"] && config["callbacks"].IsSequence()) {
    init(config, random_seed);
  }
}

SimpleTalker::~SimpleTalker()
{
  RCLCPP_INFO(this->get_logger(), "SimpleTalker node is being destroyed.");
}

int SimpleTalker::init(const YAML::Node &config, const unsigned int &random_seed)
{
  if (publishers_.size() > 0 || timers_.size() > 0) {
    return -1; // Already initialized
  }

  std::string node_name = this->get_name();
  MessageGenerator message_generator{random_seed};


  if (!config["callbacks"] || !config["callbacks"].IsSequence()) {
    RCLCPP_ERROR(this->get_logger(), "No 'callbacks' sequence found for node '%s'", node_name.c_str());
    return -1;
  }

  unsigned int callback_idx = 0;
  for (const auto &cb_config : config["callbacks"])
  {
    auto topic = cb_config["topic"].as<std::string>();
    auto frequency = cb_config["frequency"].as<double>();
    auto launch = true; // Default to not silent
    if (cb_config["launch"] && cb_config["launch"].IsScalar()) {
      launch = cb_config["launch"].as<bool>();
    }
    // message will be reserved even if launch is false because the implementation is easier.
    std_msgs::msg::String message_text;
    message_generator.generate_message(cb_config, message_text);
    messages_.emplace_back(message_text);

    bool show_count = cb_config["show_counter"].as<bool>(true);
    if (show_count) {
      messages_.back().data.append("pubs: 00000000");
    }

    if (launch) {
      // in this case, this talker will publish messages with the given  topic.
      auto publisher = this->create_publisher<std_msgs::msg::String>(topic, 10);
      publishers_.push_back(publisher);
      publishing_counters_.push_back(static_cast<unsigned int>(0));
    
      // defining the timer callback.
      auto timer_callback = [this, publisher,  callback_idx, show_count]() -> void {
        auto & message = messages_[callback_idx];

        if (show_count) {
          message.data.replace(message.data.size() - 8, 8, std::to_string(publishing_counters_[callback_idx]));
        }

        publishing_counters_[callback_idx]++;
        publisher->publish(message);
      };


      auto timer = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / frequency),
        timer_callback
      );
      timers_.push_back(timer);

      RCLCPP_INFO(this->get_logger(), "Publishing to '%s' at %f Hz", topic.c_str(), frequency);

    } else {
      // in this case, this talker will not publish messages with the given topic.
      // only timer and counter will run.
      publishing_counters_.push_back(static_cast<unsigned int>(0));

      auto timer_callback = [this, callback_idx]() -> void
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

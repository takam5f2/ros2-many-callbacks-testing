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

SimpleTalker::SimpleTalker(const std::string &default_node_name,
                           const YAML::Node &config, unsigned int random_seed,
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
  if (ptr_talker_topic_manager_ != nullptr) {
    return -1; // Already initialized
  }

  std::string node_name = this->get_name();

  std::shared_ptr<MessageGenerator> ptr_message_generator = std::make_shared<MessageGenerator>(random_seed);

  if (!config["callbacks"] || !config["callbacks"].IsSequence()) {
    RCLCPP_ERROR(this->get_logger(), "No 'callbacks' sequence found for node '%s'", node_name.c_str());
    return -1;
  }

  ptr_talker_topic_manager_ = std::make_shared<TalkerTopicManager>(ptr_message_generator, config["callbacks"].size());


  unsigned int callback_idx = 0;
  for (const auto &cb_config : config["callbacks"])
  {
    auto frequency = cb_config["frequency"].as<double>(1.0);
    auto launch = cb_config["launch"].as<bool>(true);

    // message will be reserved even if launch is false because the implementation is easier.
    ptr_talker_topic_manager_->generate_message(callback_idx, cb_config);

    if (launch) {
      auto topic = cb_config["topic"].as<std::string>("/unnamed/message");
      bool show_count = cb_config["show_counter"].as<bool>(true);

      ptr_talker_topic_manager_->create_publisher(callback_idx, this, topic);

      // defining the timer callback.
      auto timer_callback = [this, callback_idx, show_count]() -> void
      {
        ptr_talker_topic_manager_->publish_buffered_message(callback_idx, show_count);
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

      auto timer_callback = [this, callback_idx]() -> void
      {
        ptr_talker_topic_manager_->increment_publishing_count(callback_idx);
      };

      auto timer = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / frequency),
        timer_callback
      );
      timers_.push_back(timer);
      RCLCPP_INFO(this->get_logger(), "Timer set at %f Hz but not publishing messages", frequency);
    }

    callback_idx++;
  }

  return 0;
}

}

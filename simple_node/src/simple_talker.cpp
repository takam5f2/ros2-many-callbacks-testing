#include "simple_node/simple_talker.hpp"
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <std_msgs/msg/string.hpp>
#include <random>
#include <string>
#include <vector>

#include "yaml-cpp/yaml.h"

namespace simple_node
{

SimpleTalker::SimpleTalker(const std::string &default_node_name, const YAML::Node &config,
                           const rclcpp::NodeOptions &options)
  : Node(default_node_name, options)
{
  std::string node_name = this->get_name();

  RCLCPP_INFO(this->get_logger(), "%s node has been created.", node_name.c_str());

  if (options.use_intra_process_comms()) {
    RCLCPP_INFO(this->get_logger(), "Intra-process communication is enabled for node '%s'.", node_name.c_str());
  }

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
    auto launch = true; // Default to not silent
    if (cb_config["launch"] && cb_config["launch"].IsScalar()) {
      launch = cb_config["launch"].as<bool>();
    }
    bool append_count = cb_config["show_counter"].as<bool>(true);

    // message will be reserved even if launch is false because the implementation is easier.
    reserve_message(cb_config);

    if (launch) {
      // in this case, this talker will publish messages with the given  topic.
      auto publisher = this->create_publisher<std_msgs::msg::String>(topic, 10);
      publishers_.push_back(publisher);
      publishing_counters_.push_back(static_cast<unsigned int>(0));
    
      // defining the timer callback.
      auto timer_callback = [this, publisher,  callback_idx]() -> void {
        auto & message = messages_[callback_idx];
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

void SimpleTalker::reserve_message(const YAML::Node &config)
{

  std_msgs::msg::String message_text;

  MessagePattern message_pattern = static_cast<MessagePattern>(config["message_pattern"].as<unsigned int>(GIVEN_MESSAGE));

  // if the given message pattern is not valid, default to GIVEN_MESSAGE
  // if message is not set, use a default message
  if (message_pattern < GIVEN_MESSAGE || message_pattern > RANDOM) {
    message_pattern = GIVEN_MESSAGE; // Default to GIVEN_MESSAGE if invalid
  }

  // if message_pattern is GIVEN_MESSAGE, use the message from config straightforward.
  if (message_pattern == GIVEN_MESSAGE) {
    message_text.data = config["message"].as<std::string>("default message");
    messages_.emplace_back(message_text);
    return;
  }

  // if message_pattern is REPEAT or RANDOM, we need to reserve the message size.
  size_t message_byte_size = config["message_bytes"].as<size_t>();
  message_text.data.clear();
  message_text.data.reserve(message_byte_size); // reserve is not recommended for std::string, but it is acceptable because it is just a sample code.

  if (message_pattern == REPEAT) {
    std::string given_message = config["message"].as<std::string>("default message");
    repeat_message(given_message, message_byte_size, message_text.data);
    messages_.emplace_back(message_text);
    return;
  }

  if (message_pattern == RANDOM) {
    std::string given_message = config["message"].as<std::string>("default message");
    random_message(given_message, message_byte_size, message_text.data);
    messages_.emplace_back(message_text);
    return;
  }
}

void SimpleTalker::repeat_message(const std::string &given_message, const size_t &message_byte_size, std::string & message_text) {


  std::string repeated_message;
  if (given_message.size() == 0) {
    repeated_message = "default message";
  } else {
    repeated_message = given_message;
  }

  const size_t full_copies = message_byte_size / repeated_message.size();
  const size_t remaining_bytes = message_byte_size % repeated_message.size();

  for (size_t i = 0; i < full_copies; ++i) {
    message_text += repeated_message;
  }
  if (remaining_bytes > 0) {
    message_text += repeated_message.substr(0, remaining_bytes);
  }

  return;
}

void SimpleTalker::random_message(const std::string &given_message, const size_t &message_byte_size, std::string &message_text) {

  message_text = given_message;

  const std::string charset = "abcdefghijklmnopqrstuvwxyz"
                              "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
                              "0123456789";

  std::random_device rd;
  std::mt19937 generator(rd());
  std::uniform_int_distribution<> distribution(0, charset.size() - 1);


  // Randomly fill the message with characters until it reaches the desired byte size
  while (message_text.size() < message_byte_size) {
    message_text += charset[distribution(generator)];
  }

  if (message_text.size() > message_byte_size) {
    message_text.resize(message_byte_size); // Trim to the exact byte size
  }
  return;
}
}

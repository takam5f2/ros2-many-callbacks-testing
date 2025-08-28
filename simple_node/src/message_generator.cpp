#include "simple_node/message_generator.hpp"

#include <stdexcept>
#include <string>
#include <algorithm>

namespace simple_node
{

  MessageGenerator::MessageGenerator(unsigned int seed) :
  random_engine_(seed == 0 ? std::random_device{}() : seed),
  distribution_(0, charset_.size() - 1)
  {
  }

  void MessageGenerator::generate_message(const YAML::Node &config, std_msgs::msg::String &message_text)
  {
    MessagePattern message_pattern = static_cast<MessagePattern>(config["message_pattern"].as<unsigned int>(GIVEN_MESSAGE));

    if (message_pattern < GIVEN_MESSAGE || message_pattern > RANDOM) {
      message_pattern = GIVEN_MESSAGE; // Default to GIVEN_MESSAGE if invalid
    }

    std::string given_message = config["message"].as<std::string>("default message");

    if (message_pattern == GIVEN_MESSAGE) {
      message_text.data = given_message;
      return;
    }

    size_t message_byte_size = config["message_bytes"].as<size_t>(1024); // Default size
    message_text.data.clear();
    message_text.data.reserve(message_byte_size); // Reserve space for the message

    if (message_pattern == REPEAT) {
      repeat_message(given_message, message_byte_size, message_text.data);
      return;
    }

    if (message_pattern == RANDOM) {
      randomize_message(given_message, message_byte_size, message_text.data);
      return;
    }
  }

  void MessageGenerator::repeat_message(const std::string &given_message, const size_t &message_byte_size, std::string & message_text) {

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

void MessageGenerator::randomize_message(const std::string &given_message, const size_t &message_byte_size, std::string &message_text) {

  message_text = given_message;

  // Randomly fill the message with characters until it reaches the desired byte size
  while (message_text.size() < message_byte_size) {
    message_text += charset_[distribution_(random_engine_)];
  }

  if (message_text.size() > message_byte_size) {
    message_text.resize(message_byte_size); // Trim to the exact byte size
  }
  return;
}


}

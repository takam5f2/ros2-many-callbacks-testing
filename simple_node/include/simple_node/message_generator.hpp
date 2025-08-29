#ifndef __MESSAGE_GENERATOR_HPP__
#define __MESSAGE_GENERATOR_HPP__

#include <random>
#include <std_msgs/msg/string.hpp>
#include "yaml-cpp/yaml.h"

namespace simple_node
{
  enum MessagePattern : unsigned int
  {
    GIVEN_MESSAGE = 0,
    REPEAT = 1,
    RANDOM = 2,
  };

  class MessageGenerator
  {
  public:
    // pass random seed to the constructor

    explicit MessageGenerator(unsigned int seed = std::random_device{}());
    virtual ~MessageGenerator() = default;

    void generate_message(const YAML::Node &config, std_msgs::msg::String &message_text);

    void repeat_message(const std::string &given_message, const size_t &message_byte_size, std::string &message_text);
    void randomize_message(const std::string &given_message, const size_t &message_byte_size, std::string &message_text);
  private:

   // charset for random message generation
   static inline const std::string charset_ = "abcdefghijklmnopqrstuvwxyz"
                                  "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
                                  "0123456789";


    std::mt19937 random_engine_;
    std::uniform_int_distribution<> distribution_;

  };
}




#endif // __MESSAGE_GENERATOR_HPP__

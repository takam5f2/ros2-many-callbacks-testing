#ifndef __SIMPLE_TALKER_HPP__
#define __SIMPLE_TALKER_HPP__
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <vector>
#include <string>
#include "yaml-cpp/yaml.h"


namespace simple_node
{
  enum MessagePattern : unsigned int
  {
    GIVEN_MESSAGE = 0,
    REPEAT = 1,
    RANDOM = 2,
  };

  class SimpleTalker : public rclcpp::Node
  {
  public:
    SimpleTalker(const std::string &default_node_name, const YAML::Node &config = YAML::Node(),
                 const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    virtual ~SimpleTalker() override;

  private:
    int init(const YAML::Node &config);
    void reserve_message(const YAML::Node &config);
    void repeat_message(const std::string &given_message, const size_t &message_byte_size, std::string &message_text);
    void random_message(const std::string &given_message, const size_t &message_byte_size, std::string &message_text);

    // Multiple publishers for different IDs by std::vector
    std::vector<rclcpp::Publisher<std_msgs::msg::String>::SharedPtr> publishers_{};
    // Multiple timers for different IDs by std::vector
    std::vector<rclcpp::TimerBase::SharedPtr> timers_{};
    std::vector<unsigned int> publishing_counters_{}; // Store frequencies for each timer

    std::vector<std_msgs::msg::String> messages_{};
  };
}

#endif

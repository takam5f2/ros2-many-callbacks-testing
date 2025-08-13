#ifndef __SIMPLE_TALKER_HPP__
#define __SIMPLE_TALKER_HPP__
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <vector>
#include <string>
#include "yaml-cpp/yaml.h"
#include "simple_node/message_generator.hpp"

namespace simple_node
{

  class SimpleTalker : public rclcpp::Node
  {
  public:
    SimpleTalker(const std::string &default_node_name, const YAML::Node &config = YAML::Node(), unsigned int random_seed = 0,
                 const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    virtual ~SimpleTalker() override;
    int init(const YAML::Node &config, const unsigned int &random_seed = 0);
    
  private:

    // Multiple publishers for different IDs by std::vector
    std::vector<rclcpp::Publisher<std_msgs::msg::String>::SharedPtr> publishers_{};
    // Multiple timers for different IDs by std::vector
    std::vector<rclcpp::TimerBase::SharedPtr> timers_{};
    std::vector<unsigned int> publishing_counters_{}; // Store frequencies for each timer

    std::vector<std_msgs::msg::String> messages_{};
  };
}

#endif

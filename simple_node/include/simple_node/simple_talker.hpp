#ifndef __SIMPLE_TALKER_HPP__
#define __SIMPLE_TALKER_HPP__
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <vector>
#include <string>
#include "yaml-cpp/yaml.h"
#include "simple_node/message_generator.hpp"
#include "simple_node/talker_topic_manager.hpp"


namespace simple_node
{

  class SimpleTalker : public rclcpp::Node
  {
  public:
    SimpleTalker(const std::string &default_node_name,
                 const YAML::Node &config, unsigned int random_seed,
                 const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    virtual ~SimpleTalker() override;
    int init(const YAML::Node &config, const unsigned int &random_seed = 0);
    
  private:
    std::shared_ptr<TalkerTopicManager> ptr_talker_topic_manager_{nullptr};

    // Multiple timers for different IDs by std::vector
    std::vector<rclcpp::TimerBase::SharedPtr> timers_{};
  };
}

#endif

#ifndef __SIMPLE_LISTENER_HPP__
#define __SIMPLE_LISTENER_HPP__
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <vector>
#include <string>
#include "yaml-cpp/yaml.h"


namespace simple_node
{
class SimpleListener : public rclcpp::Node
{
public:
  SimpleListener(const std::string &node_name, const YAML::Node &config,
                 const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  virtual ~SimpleListener() override;
private:
  // Multiple publishers for different IDs by std::vector
  std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> subscribers_;
  std::vector<unsigned int> receiving_counters_;
};
}

#endif
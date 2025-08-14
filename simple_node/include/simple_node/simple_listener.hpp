#ifndef __SIMPLE_LISTENER_HPP__
#define __SIMPLE_LISTENER_HPP__
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <vector>
#include <string>
#include "yaml-cpp/yaml.h"

#include <agnocast/agnocast.hpp>


namespace simple_node
{
class SimpleListener : public rclcpp::Node
{
public:
  SimpleListener(const std::string &default_node_name, const YAML::Node &config = YAML::Node(),
                 const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  virtual ~SimpleListener() override;
  int init(const YAML::Node &config);
private:
  // Multiple publishers for different IDs by std::vector
  std::vector<std::shared_ptr<agnocast::Subscription<std_msgs::msg::String>>> subscribers_{};
  std::vector<unsigned int> receiving_counters_{};
};
}

#endif

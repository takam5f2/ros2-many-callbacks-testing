#include "simple_node/simple_listener.hpp"
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include "yaml-cpp/yaml.h"
#include "rclcpp_components/register_node_macro.hpp"
#include "composable_simple_node/composable_simple_node_helper.hpp"

namespace composable_simple_node
{

class ComposableSimpleListener : public simple_node::SimpleListener
{
public:
  explicit ComposableSimpleListener(const rclcpp::NodeOptions &options): 
  SimpleListener("simple_listener", YAML::Node(), options)
  {
    init(load_yaml_config(this));
  }
};
} // namespace composable_simple_node

RCLCPP_COMPONENTS_REGISTER_NODE(composable_simple_node::ComposableSimpleListener)
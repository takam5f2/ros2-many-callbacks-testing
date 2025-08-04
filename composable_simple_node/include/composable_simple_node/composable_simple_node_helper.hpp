#ifndef __COMPOSABLE_SIMPLE_NODE_HELPER_HPP__
#define __COMPOSABLE_SIMPLE_NODE_HELPER_HPP__

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <yaml-cpp/yaml.h>

namespace composable_simple_node
{

  YAML::Node load_yaml_config(rclcpp::Node* node)
  {
    std::string node_name{node->get_name()};
    node->declare_parameter<std::string>("node_config_file", "config.yaml");
    std::string node_config_file;
    node->get_parameter("node_config_file", node_config_file);

    try {
      YAML::Node config = YAML::LoadFile(node_config_file);
      return config[node_name];
    } catch (const YAML::Exception &e) {
      RCLCPP_ERROR(node->get_logger(), "Failed to load YAML file: %s", e.what());
      throw;
    }
  }
}

#endif // __COMPOSABLE_SIMPLE_NODE_HELPER_HPP__

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
    node->declare_parameter<std::string>("config_file_path", "config.yaml");
    std::string config_file_path = node->get_parameter("config_file_path").as_string();

    try {
      YAML::Node config = YAML::LoadFile(config_file_path);
      YAML::Emitter out;
      return config[node_name];
    } catch (const YAML::Exception &e) {
      RCLCPP_ERROR(node->get_logger(), "Failed to load YAML file: %s", e.what());
      throw;
    }
  }
}

#endif // __COMPOSABLE_SIMPLE_NODE_HELPER_HPP__
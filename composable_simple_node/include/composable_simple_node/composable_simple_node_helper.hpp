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
    node->declare_parameter<std::string>("executor_name", "");

    std::string node_config_file;
    std::string executor_name;
    node->get_parameter("node_config_file", node_config_file);
    node->get_parameter("executor_name", executor_name);
    

    try {
      YAML::Node config = YAML::LoadFile(node_config_file);
      // if executor_name is empty, node name is used as the first key of config.
      if (executor_name.empty()) {
        if (config[node_name]) {
          return config[node_name];
        } else {
          return YAML::Node();
        }
      }
      if (config[executor_name]) {
        for (auto node_config : config[executor_name]["nodes"]) {
          if (node_config["node_name"].as<std::string>() == node_name) {
            return node_config;
          }
        }
        return YAML::Node();
      } else {
        return YAML::Node();
      }
    } catch (const YAML::Exception &e) {
      RCLCPP_ERROR(node->get_logger(), "Failed to load YAML file: %s", e.what());
      throw;
    }
  }
}

#endif // __COMPOSABLE_SIMPLE_NODE_HELPER_HPP__

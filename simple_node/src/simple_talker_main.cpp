#include "simple_node/simple_talker.hpp"
#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"
#include <memory>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  if (argc < 3) {
    RCLCPP_ERROR(rclcpp::get_logger("main"), "Usage: simple_talker <node_name> <path_to_yaml>");
    return 1;
  }
  std::string node_name = argv[1];
  std::string yaml_path = argv[2];

  YAML::Node config = YAML::LoadFile(yaml_path);
  
  YAML::Node node_config = config[node_name];

  if (!node_config) {
    RCLCPP_ERROR(rclcpp::get_logger("main"), "Node '%s' not found in YAML file.", node_name.c_str());
    return 1;
  }
  if (node_config["type"].as<std::string>() != "talker") {
    RCLCPP_ERROR(rclcpp::get_logger("main"), "Node '%s' is not of type 'talker'.", node_name.c_str());
    return 1;
  }

  auto node = std::make_shared<simple_node::SimpleTalker>(node_name, node_config);

  
  if (node_config["executor_type"] && node_config["executor_type"].as<std::string>() == "multi_threaded") {
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor->add_node(node);
    executor->spin();

  } else {
    auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor->add_node(node);
    executor->spin();
  }

  rclcpp::shutdown();
  return 0;
}

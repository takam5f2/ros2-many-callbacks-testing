#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"

#include "simple_node/simple_listener.hpp"
#include "simple_node/simple_talker.hpp"
#include <vector>

rclcpp::Executor::SharedPtr create_executor(const YAML::Node & config)
{
  std::string executor_type = config["executor_type"].as<std::string>();
  if (executor_type == "single_threaded_executor") {
    return std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  } else if (executor_type == "multi_threaded_executor") {
    return std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("executors_loader"), "Unknown executor type: %s", executor_type.c_str());
    throw std::runtime_error("Unknown executor type");
  }
}


rclcpp::Node::SharedPtr create_simple_node(const YAML::Node & node_config, unsigned int random_seed)
{
  std::string node_name = node_config["node_name"].as<std::string>();
  std::string node_type = node_config["node_type"].as<std::string>();
  bool intra_process = node_config["intra_process"].as<bool>();

  rclcpp::NodeOptions options;
  options.use_intra_process_comms(intra_process);

  if (node_type == "simple_talker") {
    return std::make_shared<simple_node::SimpleTalker>(node_name, node_config, random_seed, options);
  } else if (node_type == "simple_listener") {
    return std::make_shared<simple_node::SimpleListener>(node_name, node_config, options);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("executors_loader"), "Unknown node type: %s", node_type.c_str());
    throw std::runtime_error("Unknown node type");
  }
}


/*
<YAML Format>
executor_name:
  executor_type: "single_threaded_executor" | "multi_threaded_executor"
  nodes:
    - node_name: "node_name_1"
      node_type: simple_talker | simple_listener
      node_namespace: <executor_name>_ns
      intra_process: true | false
      callbacks:
        - topic: /topic/message/name0
          frequency: 10
          message: "Hello from name0"
    - node_name: "node_name_1"
      node_type: simple_listener | simple_talker
      node_namespace: <executor_name>_ns
      intra_process: true | false
      callbacks:
        - topic: /topic/message/name0


<Example configuration for executors_loader>
executor_00:
  executor_type: "single_threaded_executor"
  nodes:
  - node_name: "simple_talker"
    node_type: simple_talker
    node_namespace: "executor_00_ns"
    intra_process: false
    callbacks:
      - topic: /topic/message/name0
        frequency: 10
        message: "Hello from name0"
  - node_name: "sample_listener"
    node_type: simple_listener
    node_namespace: "executor_00_ns"
    intra_process: false
    callbacks:
      - topic: /topic/message/name0
*/

int main (int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // assign executor name from arguments[1]
  // Load configuration from YAML file given by arguments[2]
  if (argc < 3) {
    RCLCPP_ERROR(rclcpp::get_logger("executors_loader"), "Usage: %s <executor name> <config_file>", argv[0]);
    return 1;
  }
  std::string executor_name = argv[1];
  std::string  config_file = argv[2];

  unsigned int random_seed = 0u;
  if (argc > 3) {
    try {
      random_seed = std::stoul(argv[3]);
    } catch (const std::invalid_argument & e) {
      random_seed = 0u; // Default seed if conversion fails
    }
  }

  YAML::Node config;
  try {
    config = YAML::LoadFile(config_file);
  } catch (const YAML::Exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("executors_loader"), "Failed to load configuration file: %s", e.what());
    return 1;
  }

  if (!config[executor_name]) {
    RCLCPP_ERROR(rclcpp::get_logger("executors_loader"), "Executor '%s' not found in configuration file", executor_name.c_str());
    return 1;
  }

  auto executor = create_executor(config[executor_name]);

  if (!executor) {
    RCLCPP_ERROR(rclcpp::get_logger("executors_loader"), "Failed to create executor for '%s'", executor_name.c_str());
    return 1;
  }


  // Create nodes based on the configuration.
  std::vector<rclcpp::Node::SharedPtr> nodes;
  for (const auto & node_config : config[executor_name]["nodes"]) {

    rclcpp::Node::SharedPtr node = create_simple_node(node_config, random_seed);
    if (!node) {
      RCLCPP_ERROR(rclcpp::get_logger("executors_loader"), "Failed to create node: %s", node_config["node_name"].as<std::string>().c_str());
      return 1;
    }
    executor->add_node(node);
    nodes.push_back(node);
  }

  executor->spin();

  rclcpp::shutdown();
  return 0;
}

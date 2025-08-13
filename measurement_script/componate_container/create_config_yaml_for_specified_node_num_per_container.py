
# yamlファイルの例は下記の通り。
"""
composable_executor_00:
  use_composable_nodes: true
  executor_type: "single_threaded_executor"
  nodes:
  - node_name: "composable_simple_talker"
    node_type: simple_talker
    intra_process: false
    callbacks:
      - topic: /topic/message/composable_name0
        frequency: 1
        message: "Hello from composable_name0"
        launch: true
  - node_name: "composable_simple_listener"
    node_type: simple_listener
    intra_process: false
    silent: true
    callbacks:
      - topic: /topic/message/composable_name0
"""
import sys
import yaml


def generate_config_for_component_container(total_node_num: int, node_num_per_component) -> dict:
    config = {}
    component_num = total_node_num // node_num_per_component
    for i in range(component_num):
        component_name = f"composable_executor_{i:02d}"
        config[component_name] = {
            "use_composable_nodes": True,
            "executor_type": "single_threaded_executor",
            "nodes": []
        }
        for j in range(node_num_per_component):
            node_index = i * node_num_per_component + j
            node_name = f"composable_simple_talker_{node_index}"
            config[component_name]["nodes"].append({
                "node_name": node_name,
                "node_type": "simple_talker",
                "intra_process": False,
                "callbacks": [{
                    "topic": f"/topic/message/composable_name{node_index}",
                    "frequency": 10,
                    "message": f"Hello from composable_name{node_index}",
                    "launch": True
                }]
            })
    left_nodes = total_node_num % node_num_per_component
    if left_nodes > 0:
        component_name = f"composable_executor_{component_num:02d}"
        config[component_name] = {
            "use_composable_nodes": True,
            "executor_type": "single_threaded_executor",
            "nodes": []
        }
        for j in range(left_nodes):
            node_index = component_num * node_num_per_component + j
            node_name = f"composable_simple_talker_{node_index}"
            config[component_name]["nodes"].append({
                "node_name": node_name,
                "node_type": "simple_talker",
                "intra_process": False,
                "callbacks": [{
                    "topic": f"/topic/message/composable_name{node_index}",
                    "frequency": 10,
                    "message": f"Hello from composable_name{node_index}",
                    "launch": True
                }]
            })
    return config


def main():
    # 引数でノード数を取得
    if len(sys.argv) != 3:
        print(f"Usage: python {sys.argv[1]} <total_node_num> <node_num_per_component>")
        sys.exit(1)
    
    try:
        total_node_num = int(sys.argv[1])
        node_num_per_component = int(sys.argv[2])
    except ValueError:
        print("Error: total_node_num and node_num_per_component must be integers.")
        sys.exit(1)
    config = generate_config_for_component_container(total_node_num, node_num_per_component)
    # YAMLファイルに書き出し
    output_file = "config.yaml"
    with open(output_file, 'w') as file:
        yaml.dump(config, file, default_flow_style=False, allow_unicode=True)
    print(f"Configuration written to {output_file}")


if __name__ == "__main__":
    main()


# yamlファイルの例は下記の通り。
"""
# 0: sample listener and talker
# callback number is 2 for each talker and listener
a_talker:
  type: talker
  executor_type: single_threaded
  callbacks:
    - topic: /topic/message/name0
      frequency: 10
      message: "Hello from name0"

a_listener:
  type: listener
  executor_type: single_threaded
  silent: true
  callbacks:
    - topic: /topic/message/name0
"""
import sys
import yaml


def generate_config_for_pub_sub(node_num: int, topic_num_per_node: int) -> dict:
    config = {}
    topic_index = 0
    for i in range(node_num//2):
        # topic index range
        topic_index_range = range(topic_index, topic_index + topic_num_per_node)
        topic_index += topic_num_per_node
        # talkerの設定
        talker_name = f"talker_{i}"
        config[talker_name] = {
            "type": "talker",
            "executor_type": "single_threaded",
            "callbacks": []
        }
        for j in topic_index_range:
            config[talker_name]["callbacks"].append({
                "topic": f"/topic/message/name{j}",
                "frequency": 1,
                "message": f"Hello from name{j}"
            })
        # listenerの設定
        listener_name = f"listener_{i}"
        config[listener_name] = {
            "type": "listener",
            "executor_type": "single_threaded",
            "silent": True,
            "callbacks": []
        }
        for j in topic_index_range:
            config[listener_name]["callbacks"].append({
                "topic": f"/topic/message/name{j}"
            })
    return config


def main():
    # 引数でノード数を取得
    if len(sys.argv) != 3:
        print("Usage: python specified_node_num.py <node_num> <topic_num>")
        sys.exit(1)
    
    node_num = int(sys.argv[1])
    topic_num = int(sys.argv[2])
    config = generate_config_for_pub_sub(node_num, topic_num)
    # YAMLファイルに書き出し
    output_file = "config.yaml"
    with open(output_file, 'w') as file:
        yaml.dump(config, file, default_flow_style=False, allow_unicode=True)
    print(f"Configuration written to {output_file}")


if __name__ == "__main__":
    main()


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


def generate_config_for_pub_sub(node_num: int) -> dict:
    config = {}
    for i in range(node_num//2):
        # talkerの設定
        talker_name = f"talker_{i}"
        config[talker_name] = {
            "type": "talker",
            "executor_type": "single_threaded",
            "callbacks": [
                {
                    "topic": f"/topic/message/name{i}",
                    "frequency": 10,
                    "message": f"Hello from name{i}"
                }
            ]
        }
        # listenerの設定
        listener_name = f"listener_{i}"
        config[listener_name] = {
            "type": "listener",
            "executor_type": "single_threaded",
            "silent": True,
            "callbacks": [
                {
                    "topic": f"/topic/message/name{i}"
                }
            ]
        }
    return config


def generate_config_for_pub_only(node_num: int) -> dict:
    config = {}
    for i in range(node_num):
        # talkerの設定
        talker_name = f"talker_{i}"
        config[talker_name] = {
            "type": "talker",
            "executor_type": "single_threaded",
            "callbacks": [
                {
                    "topic": f"/topic/message/name{i}",
                    "frequency": 10,
                    "message": f"Hello from name{i}"
                }
            ]
        }
    return config


def generate_config_for_no_publisher(node_num: int) -> dict:
    config = {}
    for i in range(node_num):
        # talkerの設定
        talker_name = f"talker_{i}"
        config[talker_name] = {
            "type": "talker",
            "executor_type": "single_threaded"
        }
    return config


def main():
    # 引数でノード数を取得
    if len(sys.argv) != 3:
        print("Usage: python specified_node_num.py <node_num> [pub_sub|pub_only|no_publisher]")
        sys.exit(1)
    
    node_num = int(sys.argv[1])
    mode = sys.argv[2]
    if mode == "pub_sub":
        config = generate_config_for_pub_sub(node_num)
    elif mode == "pub_only":
        config = generate_config_for_pub_only(node_num)
    elif mode == "no_publisher":
        config = generate_config_for_no_publisher(node_num)
    else:
        print("Invalid mode. Use 'pub_sub', 'pub_only', or 'no_publisher'.")
        sys.exit(1)
    # YAMLファイルに書き出し
    output_file = "config.yaml"
    with open(output_file, 'w') as file:
        yaml.dump(config, file, default_flow_style=False, allow_unicode=True)
    print(f"Configuration written to {output_file}")


if __name__ == "__main__":
    main()

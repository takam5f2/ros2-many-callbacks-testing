
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


def generate_config_for_pub_sub(node_num: int, topic_size_byte_per_sec: int) -> dict:
    config = {}
    message_size_byte = topic_size_byte_per_sec // 10  # Assuming frequency is fixed at 10 Hz and 1 char = 1 byte = 8 bits
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
                    "message": f"Hello from name{i}",
                    "message_pattern": 1, # 0: As-is 1: Repeat, 2: Random
                    "message_bytes": message_size_byte, # size of the message in bytes
                    "show_count": False # 末尾に publish 回数を載せるか否か
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


def main():
    # 引数でノード数を取得
    if len(sys.argv) != 3:
        print("Usage: python specified_node_num.py <node_num> <topic_size_byte_per_sec>")
        sys.exit(1)
    
    node_num = int(sys.argv[1])
    topic_size_byte_per_sec = int(sys.argv[2])
    config = generate_config_for_pub_sub(node_num, topic_size_byte_per_sec)
    # YAMLファイルに書き出し
    output_file = "config.yaml"
    with open(output_file, 'w') as file:
        yaml.dump(config, file, default_flow_style=False, allow_unicode=True)
    print(f"Configuration written to {output_file}")


if __name__ == "__main__":
    main()

import yaml # PyYAMLライブラリが必要です (pip install pyyaml)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# OpaqueFunction内で実行される関数
# contextオブジェクトを介してLaunchの引数にアクセスできます
def launch_setup(context, *args, **kwargs):
    
    # 1. LaunchConfigurationからYAMLファイルのパスを取得
    config_file_path = LaunchConfiguration('node_config_file').perform(context)


    # 2. YAMLファイルを読み込む
    with open(config_file_path, 'r') as file:
        config = yaml.safe_load(file)


    """
    a_node:
      type: listener
      executor_type: single_threaded
      callbacks:
        - topic: /topic/message/name0
        - topic: /topic/message/name1
    b_node:...
    """
    # 3. 読み込んだ設定から Node オブジェクトのリストを生成
    nodes_to_launch = []
    for node_name, node_info in config.items():

        node_type = None
        if node_info['type'] == 'listener':
            node_type = 'simple_listener'
        elif node_info['type'] == 'talker':
            node_type = 'simple_talker'

        nodes_to_launch.append(
            Node(
                package='simple_node',
                executable=node_type,
                name=node_name,
                arguments=[node_name, config_file_path],
                output='screen',
            )
        )


    # Nodeオブジェクトのリストを返す
    return nodes_to_launch


def generate_launch_description():
    
    # 処理するYAMLファイルパスを受け取るための引数を宣言
    config_file_arg = DeclareLaunchArgument(
        'node_config_file',
        description='Nodes to launch, described in a YAML file.'
    )

    # OpaqueFunctionを使って、Launch実行時に上記のlaunch_setup関数を呼び出す
    # これにより、ユーザーが指定したファイルパスに基づいて動的に処理できる
    dynamic_nodes = OpaqueFunction(function=launch_setup)

    return LaunchDescription([
        config_file_arg,
        dynamic_nodes
    ])
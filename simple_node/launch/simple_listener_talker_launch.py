import yaml
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# OpaqueFunction内で実行される関数
# contextオブジェクトを介してLaunchの引数にアクセスできます
def launch_setup(context, *args, **kwargs):
    
    # 1. LaunchConfigurationからYAMLファイルのパスを取得
    config_file_path = LaunchConfiguration('node_config_file').perform(context)
    random_seed = LaunchConfiguration('random_seed').perform(context)

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
                arguments=[node_name, config_file_path, random_seed],
                output='screen',
            )
        )


    # Nodeオブジェクトのリストを返す
    return nodes_to_launch


def generate_launch_description():
    
    # 処理するYAMLファイルパスを受け取るための引数を宣言
    package_share_dir = get_package_share_directory('simple_node')
    default_config_path = package_share_dir + '/config/node_settings.yaml'
    config_file_arg = DeclareLaunchArgument(
        'node_config_file',
        default_value=default_config_path,
        description='Nodes to launch, described in a YAML file.'
    )

    random_seed_arg = DeclareLaunchArgument(
        'random_seed',
        default_value='0',
        description='Random seed to feed. default value means no use of random seed.'
    )

    # OpaqueFunctionを使って、Launch実行時に上記のlaunch_setup関数を呼び出す
    # これにより、ユーザーが指定したファイルパスに基づいて動的に処理できる
    dynamic_nodes = OpaqueFunction(function=launch_setup)

    return LaunchDescription([
        config_file_arg,
        random_seed_arg,
        dynamic_nodes
    ])

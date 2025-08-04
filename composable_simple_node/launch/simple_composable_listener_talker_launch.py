import yaml
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

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

        plugin = None
        if node_info['type'] == 'listener':
            plugin = 'composable_simple_node::ComposableSimpleListener'
        elif node_info['type'] == 'talker':
            plugin = 'composable_simple_node::ComposableSimpleTalker'
        else:
            print(f"Unknown node type '{node_info['type']}' for node '{node_name}'. Skipping...")
            continue

        nodes_to_launch.append(
            ComposableNode(
                package='composable_simple_node',
                plugin=plugin,
                name=node_name,
                parameters=[
                    {'config_file_path': config_file_path},
                ],
            )
        )

    # 4. ComposableNodeContainerを作成し、上記のノードを登録
    container = ComposableNodeContainer(
        name='composable_node_container',
        namespace='/composable_nodes',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=nodes_to_launch,
        output='screen',
        prefix=['xterm -e gdb -ex run --args' ],
    )

    # Nodeオブジェクトのリストを返す
    return  [container]


def generate_launch_description():
    
    # 処理するYAMLファイルパスを受け取るための引数を宣言
    package_share_dir = get_package_share_directory('simple_node')
    default_config_path = package_share_dir + '/config/node_settings.yaml'
    config_file_arg = DeclareLaunchArgument(
        'node_config_file',
        default_value=default_config_path,
        description='Nodes to launch, described in a YAML file.'
    )

    # OpaqueFunctionを使って、Launch実行時に上記のlaunch_setup関数を呼び出す
    # これにより、ユーザーが指定したファイルパスに基づいて動的に処理できる
    dynamic_nodes = OpaqueFunction(function=launch_setup)

    return LaunchDescription([
        config_file_arg,
        dynamic_nodes
    ])

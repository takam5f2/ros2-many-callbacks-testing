import yaml
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def get_component_container(executor_name, executor_property, config_file_path):
  node_list = []
  for node in executor_property['nodes']:
        node_name = node['node_name']
        node_type = node['node_type']
        intra_process = node.get('intra_process', False)

        if node_type == 'simple_talker':
            plugin_type = 'composable_simple_node::ComposableSimpleTalker'
        elif node_type == 'simple_listener':
            plugin_type = 'composable_simple_node::ComposableSimpleListener'
        else:
            raise ValueError(f"Unsupported node type: {node_type}")

        # ここでコンポーザブルノードを定義
        node_list.append(
            ComposableNode (
                package='composable_simple_node',
                plugin=plugin_type,
                name=node_name,
                namespace=executor_name,
                parameters=[{'node_config_file': config_file_path,
                             'executor_name': executor_name},
                             ],
                extra_arguments=[{'use_intra_process_comms': intra_process}],
            )
        )

  return ComposableNodeContainer(
      name=executor_name + '_container',
      namespace=executor_name,
      package='rclcpp_components',
      executable='component_container',
      composable_node_descriptions=node_list,
      output='screen',
  )


def launch_setup(context, *args, **kwargs):
    
    # 1. LaunchConfigurationからYAMLファイルのパスを取得
    config_file_path = LaunchConfiguration('executor_config_file').perform(context)


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
    executors_to_launch = []
    component_container_to_launch = []

    for executor_name, executor_property in config.items():
        use_composable_nodes = executor_property.get('use_composable_nodes', False)
        
        if not use_composable_nodes:
          executors_to_launch.append(
              Node(
                  package='executors_loader',
                  executable='executors_loader',
                  namespace=executor_name,
                  arguments=[executor_name, config_file_path],
                  output='screen',
              )
          )
        else:
          component_container_to_launch.append(get_component_container(executor_name, executor_property, config_file_path))

    executors_to_launch.extend(component_container_to_launch)

    # Nodeオブジェクトのリストを返す
    return executors_to_launch


def generate_launch_description():
    
    # 処理するYAMLファイルパスを受け取るための引数を宣言
    package_share_dir = get_package_share_directory('executors_loader')
    default_config_path = package_share_dir + '/config/executor_setting.yaml'
    config_file_arg = DeclareLaunchArgument(
        'executor_config_file',
        default_value=default_config_path,
        description='Nodes to launch, described in a YAML file.'
    )

    # OpaqueFunctionを使って、Launch実行時に上記のlaunch_setup関数を呼び出す
    # これにより、ユーザーが指定したファイルパスに基づいて動的に処理できる
    executor_loader_launch = OpaqueFunction(function=launch_setup)

    return LaunchDescription([
        config_file_arg,
        executor_loader_launch
    ])

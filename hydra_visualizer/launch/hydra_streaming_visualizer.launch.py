from launch_ros.parameter_descriptions import ParameterFile
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

def generate_launch_description():
    # Declare all launch arguments
    return LaunchDescription([
        DeclareLaunchArgument('visualizer_use_zmq', default_value='false', description='Use ZMQ to receive scene graphs'),
        DeclareLaunchArgument('visualizer_zmq_ip', default_value='127.0.0.1', description='ZMQ IP to listen on'),
        DeclareLaunchArgument('visualizer_zmq_port', default_value='8001', description='ZMQ port to listen on'),
        DeclareLaunchArgument(
            'visualizer_zmq_url',
            default_value=PathJoinSubstitution([
                'tcp://',
                LaunchConfiguration('visualizer_zmq_ip'),
                ':',
                LaunchConfiguration('visualizer_zmq_port')
            ]),
            description='Full ZMQ URL'
        ),
        DeclareLaunchArgument('graph_type', default_value='GraphFromRos'),
        SetLaunchConfiguration('graph_type', 'GraphTypeZmq', condition=IfCondition(LaunchConfiguration('visualizer_use_zmq'))),
        DeclareLaunchArgument('visualizer_frame', default_value='map', description='Visualizer frame'),
        DeclareLaunchArgument('color_mesh_by_label', default_value='true', description='Color mesh by label'),
        DeclareLaunchArgument(
            'visualizer_config_path',
            default_value=PathJoinSubstitution([
                FindPackageShare('hydra_visualizer'),
                'config',
                'visualizer_config.yaml'
            ]),
            description='Path to visualizer config file'
        ),
        DeclareLaunchArgument(
            'visualizer_plugins_path',
            default_value=PathJoinSubstitution([
                FindPackageShare('hydra_visualizer'),
                'config',
                'visualizer_plugins.yaml'
            ]),
            description='Path to visualizer plugins file'
        ),
        DeclareLaunchArgument(
            'external_plugins_path',
            default_value=PathJoinSubstitution([
                FindPackageShare('hydra_visualizer'),
                'config',
                'external_plugins.yaml'
            ]),
            description='Path to external plugins file'
        ),
        DeclareLaunchArgument('verbosity', default_value='0', description='Verbosity level'),
        DeclareLaunchArgument('visualizer_debug', default_value='false', description='Enable debugger for visualizer'),
        DeclareLaunchArgument('visualizer_roslog_destination', default_value='screen', description='ROS log destination'),
        DeclareLaunchArgument('visualizer_launch_prefix', default_value='', description='Launch prefix for debugging'),
        SetLaunchConfiguration('visualizer_launch_prefix', 'gdb -ex run --args', 
                               condition=IfCondition(LaunchConfiguration('visualizer_debug'))),
        # Node configuration
        Node(
            package='hydra_visualizer',
            executable='hydra_visualizer_node',
            # name='hydra_dsg_visualizer',
            output=LaunchConfiguration('visualizer_roslog_destination'),
            prefix=LaunchConfiguration('visualizer_launch_prefix'),
            arguments=[
                '-alsologtostderr',
                '-colorlogtostderr',
                ['-v=', LaunchConfiguration('verbosity')]
            ],
            parameters=[
                {'visualizer_frame': LaunchConfiguration('visualizer_frame')},
                {'graph.type': LaunchConfiguration('graph_type')},
                {'graph.url': LaunchConfiguration('visualizer_zmq_url')},
                ParameterFile(LaunchConfiguration('visualizer_config_path')),
                ParameterFile(LaunchConfiguration('visualizer_plugins_path'), allow_substs=True),
                ParameterFile(LaunchConfiguration('external_plugins_path'))
            ]
        )
    ])

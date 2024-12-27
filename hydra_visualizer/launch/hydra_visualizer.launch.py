import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch arguments
    return LaunchDescription([
        # Declare all the launch arguments
        DeclareLaunchArgument('scene_graph', default_value="''", description='Scene graph filename to show'),
        DeclareLaunchArgument('scene_graph_dir', 
                              default_value=PathJoinSubstitution(['/workspaces/ros2_ws/src',
                                            'hydra', 'output']), 
                              description='Directory of scene graph file'),
        DeclareLaunchArgument('publish_view_transforms', default_value='false', description='Publish view transforms'),
        DeclareLaunchArgument('visualizer_ns', default_value='hydra_visualizer', description='Visualizer namespace'),
        DeclareLaunchArgument('visualizer_frame', default_value='map', description='Visualizer frame'),
        DeclareLaunchArgument('color_mesh_by_label', default_value='true'),
        DeclareLaunchArgument('visualizer_config_path', 
                              default_value=PathJoinSubstitution([FindPackageShare('hydra_visualizer'),
                                                    'config/visualizer_config.yaml'])),
        DeclareLaunchArgument('visualizer_plugins_path', 
                              default_value=PathJoinSubstitution([FindPackageShare('hydra_visualizer'),
                                                    'config/visualizer_plugins.yaml'])),
        DeclareLaunchArgument('external_plugins_path', 
                              default_value=PathJoinSubstitution([FindPackageShare('hydra_visualizer'),
                                                    'config/external_plugins.yaml'])),
        DeclareLaunchArgument('verbosity', default_value='0', description='Visualizer verbosity'),
        DeclareLaunchArgument('visualizer_debug', default_value='false'),
        DeclareLaunchArgument('start_rviz', default_value='true', description='Automatically start RViz'),
        
        # Derived arguments
        DeclareLaunchArgument(
            'scene_graph_path', 
            default_value=PathJoinSubstitution([
                LaunchConfiguration('scene_graph_dir'), 
                LaunchConfiguration('scene_graph')
            ])
        ),
        DeclareLaunchArgument('visualizer_launch_prefix', default_value='', description='Launch prefix for debugging'),
        SetLaunchConfiguration('visualizer_launch_prefix', 'gdb -ex run --args', 
                               condition=IfCondition(LaunchConfiguration('visualizer_debug'))),


        # Include RViz launch file conditionally
        GroupAction(
            condition=IfCondition(LaunchConfiguration('start_rviz')),
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution([
                            FindPackageShare('hydra_visualizer'),
                            'launch',
                            'hydra_rviz.launch.py'
                        ])
                    ),
                    launch_arguments={
                        'rviz_path': PathJoinSubstitution([
                            FindPackageShare('hydra_visualizer'),
                            'rviz',
                            'hydra_visualizer.rviz'
                        ])
                    }.items()
                )
            ]
        ),

        # Hydra visualizer node
        Node(
            package='hydra_visualizer',
            executable='hydra_visualizer_node',
            name=LaunchConfiguration('visualizer_ns'),
            output='screen',
            prefix=LaunchConfiguration('visualizer_launch_prefix'),
            parameters=[
                {'visualizer_frame': LaunchConfiguration('visualizer_frame')},
                {'graph.type': 'GraphFromFile'},
                {'graph.filepath': LaunchConfiguration('scene_graph_path')},
                PathJoinSubstitution([
                    FindPackageShare('hydra_visualizer'),
                    'config',
                    'visualizer_config.yaml'
                ]),
                PathJoinSubstitution([
                    FindPackageShare('hydra_visualizer'),
                    'config',
                    'visualizer_plugins.yaml'
                ]),
                PathJoinSubstitution([
                    FindPackageShare('hydra_visualizer'),
                    'config',
                    'external_plugins.yaml'
                ])
            ],
            arguments=[
                '-alsologtostderr',
                '-colorlogtostderr',
                ['-v=', LaunchConfiguration('verbosity')]
            ],
            remappings=[]
        ),

        # Group for view transforms
        GroupAction(
            condition=IfCondition(LaunchConfiguration('publish_view_transforms')),
            actions=[
                # Orthographic view transform
                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name='ortho_tf_publisher',
                    arguments=[
                        '0', '0', '0', 
                        '0.707', '0', '0', '0.707',
                        LaunchConfiguration('visualizer_frame'), 'rviz_ortho'
                    ]
                ),
                # Top-down view transform
                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name='topdown_tf_publisher',
                    arguments=[
                        '0', '0', '0', 
                        '0', '0.707', '0', '0.707',
                        LaunchConfiguration('visualizer_frame'), 'rviz_topdown'
                    ]
                )
            ]
        )
    ])

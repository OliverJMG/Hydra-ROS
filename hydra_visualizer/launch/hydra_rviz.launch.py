import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare arguments
    # rviz_verbose = LaunchConfiguration('rviz_verbose')
    # rviz_dir = LaunchConfiguration('rviz_dir')
    # rviz_file = LaunchConfiguration('rviz_file')
    # rviz_path = PathJoinSubstitution([rviz_dir, rviz_file])

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'rviz_verbose', 
            default_value='false', 
            description='Set RViz to log to stdout'
        ),
        DeclareLaunchArgument(
            'rviz_dir',
            default_value=PathJoinSubstitution([FindPackageShare('hydra_visualizer'), 'rviz']),
            description='Top-level RViz directory'
        ),
        DeclareLaunchArgument(
            'rviz_file',
            default_value='hydra_visualizer.rviz',
            description='RViz file relative to the directory'
        ),
        DeclareLaunchArgument(
            'rviz_path',
            default_value=PathJoinSubstitution([LaunchConfiguration('rviz_dir'), 
                                                LaunchConfiguration('rviz_file')])
        ),

        # Define the RViz node
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rviz_path')]
        ),
    ])

import launch
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'frame_id', 
            default_value='map', 
            description='Frame id to use'
        ),
        DeclareLaunchArgument(
            'draw_labels', 
            default_value='true', 
            description='Draw region labels'
        ),
        DeclareLaunchArgument(
            'region_filepath', 
            default_value="''", 
            description='Path to region file'
        ),

        Node(
            package='hydra_visualizer',
            executable='region_publisher',
            name='region_publisher',
            parameters=[
                {
                    'frame_id': launch.substitutions.LaunchConfiguration('frame_id'),
                    'region_filepath': launch.substitutions.LaunchConfiguration('region_filepath'),
                    'draw_labels': launch.substitutions.LaunchConfiguration('draw_labels'),
                    'label_offset': 2.0,
                    'fill_polygons': False,
                    'use_boundary_color': True,
                    'line_width': 0.20,
                    'label_scale': 1.0,
                }
            ]
        ),
    ])

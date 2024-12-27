from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetLaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap, SetUseSimTime
from launch_ros.substitutions import FindPackageShare
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    return LaunchDescription([
        # Declare Launch Arguments
        DeclareLaunchArgument('sim_time_required', default_value='true'),
        SetUseSimTime(True, condition=IfCondition(LaunchConfiguration('sim_time_required'))),
        DeclareLaunchArgument('use_gt_frame', default_value='true'),
        DeclareLaunchArgument('use_gt_semantics', default_value='true'),
        DeclareLaunchArgument('color_mesh_by_label', default_value='false'),
        DeclareLaunchArgument('use_single_channel_label_image', default_value='false'),
        DeclareLaunchArgument('use_gnn_descriptors', default_value='false'),
        DeclareLaunchArgument('use_static_tfs', default_value='true'),
        DeclareLaunchArgument('use_prerecorded_semantics', default_value='false'),
        DeclareLaunchArgument('use_openset_features', default_value='false'),
        DeclareLaunchArgument('start_hydra', default_value='true'),
        DeclareLaunchArgument('start_visualizer', default_value='true'),
        DeclareLaunchArgument('start_rviz', default_value='true'),

        DeclareLaunchArgument('robot_frame', default_value='base_link', description='Robot base frame (i.e., robot pose)'),
        DeclareLaunchArgument('odom_frame', default_value='odom', description='Robot map frame'),
        DeclareLaunchArgument('sensor_frame', default_value='left_cam', description='Robot sensor frame'),
        DeclareLaunchArgument('lcd_config_name', default_value='', description='LCD config name'),
        DeclareLaunchArgument('labelspace_name', default_value='', description='Label space name'),
        # Derived Arguments
        SetLaunchConfiguration('robot_frame', 'base_link_gt', condition=IfCondition(LaunchConfiguration('use_gt_frame'))),
        SetLaunchConfiguration('robot_frame', 'base_link_kimera', condition=UnlessCondition(LaunchConfiguration('use_gt_frame'))),
        SetLaunchConfiguration('odom_frame', 'world', condition=IfCondition(LaunchConfiguration('use_gt_frame'))),
        SetLaunchConfiguration('odom_frame', 'odom', condition=UnlessCondition(LaunchConfiguration('use_gt_frame'))),
        SetLaunchConfiguration('sensor_frame', 'left_cam', condition=IfCondition(LaunchConfiguration('use_gt_frame'))),
        SetLaunchConfiguration('sensor_frame', 'left_cam_kimera', condition=UnlessCondition(LaunchConfiguration('use_gt_frame'))),
        SetLaunchConfiguration('lcd_config_name', 'uhumans2_gnn.yaml', condition=IfCondition(LaunchConfiguration('use_gnn_descriptors'))),
        SetLaunchConfiguration('lcd_config_name', 'uhumans2.yaml', condition=UnlessCondition(LaunchConfiguration('use_gnn_descriptors'))),
        SetLaunchConfiguration('labelspace_name', 'uhumans2_apartment', condition=IfCondition(LaunchConfiguration('use_gt_semantics'))),
        SetLaunchConfiguration('labelspace_name', 'ade20k_mp3d', condition=UnlessCondition(LaunchConfiguration('use_gt_semantics'))),

        DeclareLaunchArgument("rgb_topic", default_value="/tesse/left_cam/rgb/image_raw"),
        DeclareLaunchArgument("rgb_info_topic", default_value="/tesse/left_cam/camera_info"),
        DeclareLaunchArgument("depth_topic", default_value="/tesse/depth_cam/mono/image_raw"),
        DeclareLaunchArgument("label_topic", default_value="semantic_inference/semantic/image_raw"),
        GroupAction([
            SetLaunchConfiguration("label_topic", "/tesse/seg_cam/converted/image_raw", 
                    condition=IfCondition(LaunchConfiguration('use_single_channel_label_image'))),
            SetLaunchConfiguration("label_topic", '/tesse/seg_cam/rgb/image_raw', 
                    condition=UnlessCondition(LaunchConfiguration('use_single_channel_label_image')))
        ], condition=IfCondition(LaunchConfiguration('use_gt_semantics')), scoped=False),

        # Static TFs Group
        GroupAction([
            IncludeLaunchDescription(
                XMLLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare('hydra_ros'), 'launch', 'static_tfs', 'uhumans2_static_tfs.xml'])
                )
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='fake_world_tf',
                arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'world'],
                condition=IfCondition(LaunchConfiguration('use_gt_frame'))
            ),
        ], condition=IfCondition(LaunchConfiguration('use_static_tfs'))),

        # Semantic Inference Group
        GroupAction([
            IncludeLaunchDescription(PathJoinSubstitution([FindPackageShare('semantic_inference_ros'), 'launch', 'semantic_inference.launch'])),
        ], condition=UnlessCondition(LaunchConfiguration('use_gt_semantics'))),

        # OpenSet Features Group
        GroupAction([
            IncludeLaunchDescription(PathJoinSubstitution([FindPackageShare('semantic_inference_ros'), 'launch', 'clip_publisher.launch'])),
        ], condition=IfCondition(LaunchConfiguration('use_openset_features'))),

        # Hydra Group
        GroupAction([
            SetRemap("/input/left_cam/depth_registered/image_rect",
                     LaunchConfiguration('depth_topic')),
            SetRemap("/input/left_cam/rgb/image_raw",
                     LaunchConfiguration('rgb_topic')),
            SetRemap("/input/left_cam/rgb/camera_info",
                     LaunchConfiguration('rgb_info_topic')),
            SetRemap("/input/left_cam/semantic/image_raw",
                     LaunchConfiguration('label_topic')),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare('hydra_ros'), 'launch', 'hydra.launch.py'])
                ),
                launch_arguments={'dataset_name': 'uhumans2'}.items()
            )
        ], launch_configurations={
            'robot_frame' : LaunchConfiguration('robot_frame')
        },
        condition=IfCondition(LaunchConfiguration('start_hydra'))),

        # Visualizer Group
        GroupAction([
            SetRemap("hydra_dsg_visualizer/dsg",
                     "hydra_ros_node/backend/dsg"),
            SetRemap("hydra_dsg_visualizer/feature",
                     "hydra_ros_node/input/left_cam/feature"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare('hydra_visualizer'), 'launch', 'hydra_streaming_visualizer.launch.py'])
                ),
                launch_arguments={'visualizer_frame': 'map'}.items()
            ),
        ], condition=IfCondition(LaunchConfiguration('start_visualizer'))),

        # Rviz Group
        GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare('hydra_visualizer'), 'launch', 'hydra_rviz.launch.py']),
                ),
                launch_arguments={'rviz_dir': PathJoinSubstitution([FindPackageShare('hydra_ros'), 'rviz']),
                                  'rviz_file': 'uhumans2.rviz'}.items()
            )
        ], condition=IfCondition(LaunchConfiguration('start_rviz')))
    ])

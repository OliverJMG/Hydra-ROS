from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([

        DeclareLaunchArgument('semantic_map_path', 
                              default_value=PathJoinSubstitution([
                                   FindPackageShare('hydra_ros'),
                                   'config/color/uhumans2_apartment.csv'
                               ])),
        DeclareLaunchArgument('semantic_label_remap_filepath', default_value="''"),
        DeclareLaunchArgument('exit_after_clock', default_value='false'),
        DeclareLaunchArgument('log_path',
                               default_value=PathJoinSubstitution([
                                   '/workspaces/ros2_ws/src/hydra', 
                                   'output', LaunchConfiguration('dataset_name')
                               ])),
        DeclareLaunchArgument('log_timing_incrementally', default_value='false'),
        DeclareLaunchArgument('disable_timer_output', default_value='true'),
        DeclareLaunchArgument('timing_disabled', default_value='false'),

        # Frontend configuration
        DeclareLaunchArgument('enable_frontend_output', default_value='true'),
        DeclareLaunchArgument('enable_places', default_value='true'),
        DeclareLaunchArgument('bounding_box_type', default_value='AABB'),
        DeclareLaunchArgument('use_gt_frame', default_value='true'),
        DeclareLaunchArgument('pose_graph_tracker_type',
                               default_value='RosPoseGraphs'),
        SetLaunchConfiguration('pose_graph_tracker_type', 'PoseGraphFromOdom', 
                               condition=IfCondition(LaunchConfiguration('use_gt_frame'))),

        # Backend configuration
        DeclareLaunchArgument('add_initial_prior', default_value='true'),
        DeclareLaunchArgument('optimize_on_lc', default_value='true'),
        DeclareLaunchArgument('zmq_ip', default_value='127.0.0.1'),
        DeclareLaunchArgument('zmq_send_url',
                               default_value=PathJoinSubstitution([
                                   'tcp://', LaunchConfiguration('zmq_ip'), ':8001'
                               ])),
        DeclareLaunchArgument('zmq_recv_url',
                               default_value=PathJoinSubstitution([
                                   'tcp://', LaunchConfiguration('zmq_ip'), ':8002'
                               ])),
        DeclareLaunchArgument('zmq_send_mesh', default_value='true'),

        # LCD pipeline configuration
        DeclareLaunchArgument('enable_dsg_lcd', default_value='false'),
        DeclareLaunchArgument('lcd_use_bow_vectors', default_value='true'),
        DeclareLaunchArgument('use_gnn_descriptors', default_value='false'),
        DeclareLaunchArgument('lcd_gnn_object_model',
                               default_value=PathJoinSubstitution([
                                   '/workspaces/ros2_ws/src/hydra', 
                                   'models', 'lcd', 'object_gnn.onnx'
                               ])),
        DeclareLaunchArgument('lcd_gnn_places_model',
                               default_value=PathJoinSubstitution([
                                   '/workspaces/ros2_ws/src/hydra', 
                                   'models', 'lcd', 'place_gnn.onnx'
                               ])),
        DeclareLaunchArgument('lcd_gnn_label_embeddings', default_value="''"),

        # Openset feature configuration
        DeclareLaunchArgument('use_openset_features', default_value='false'),
        DeclareLaunchArgument('feature_receiver_type',
                               default_value=""),
        SetLaunchConfiguration('feature_receiver_type', 'FeatureReceiver', 
                               condition=IfCondition(LaunchConfiguration('use_openset_features'))),
        
    ])

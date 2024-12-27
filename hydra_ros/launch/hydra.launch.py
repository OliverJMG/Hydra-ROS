from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node, SetParametersFromFile
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('robot_id', default_value='0', description='Unique robot identifier'),
        # DeclareLaunchArgument('robot_frame', default_value='base_link', description='Robot base frame (i.e., robot pose)'),
        # DeclareLaunchArgument('odom_frame', default_value='odom', description='Robot map frame'),
        DeclareLaunchArgument('map_frame', default_value='map', description='Backend scene graph frame'),
        DeclareLaunchArgument('dataset_name', default_value='uhumans2', description='Dataset name'),
        # DeclareLaunchArgument('labelspace_name', default_value='uhumans2_apartment', description='Label space name'),
        DeclareLaunchArgument('hydra_ros_ns', default_value='hydra_ros_node', description='Hydra ROS node namespace'),
        DeclareLaunchArgument('sensor_min_range', default_value='0.1', description='Minimum sensor range in meters'),
        DeclareLaunchArgument('sensor_max_range', default_value='5.0', description='Maximum sensor range in meters'),
        DeclareLaunchArgument('input_config_file', default_value=[LaunchConfiguration('dataset_name'), '.yaml']), 
        DeclareLaunchArgument('hydra_config_path', default_value=PathJoinSubstitution([
                                   FindPackageShare('hydra_ros'),
                                   'config/pipelines/',
                                   LaunchConfiguration('input_config_file')
                               ]), 
                               description='General configuration file for Hydra'),
        DeclareLaunchArgument('input_config_path',
                               default_value=PathJoinSubstitution([
                                   FindPackageShare('hydra_ros'),
                                   'config/datasets/',
                                   LaunchConfiguration('input_config_file')
                               ])),
        DeclareLaunchArgument('mesh_segmenter_sinks',
                               default_value=PathJoinSubstitution([
                                   FindPackageShare('hydra_ros'),
                                   'config/sinks/mesh_segmenter_sinks.yaml'
                               ])),
        DeclareLaunchArgument('gvd_places_sinks',
                               default_value=PathJoinSubstitution([
                                   FindPackageShare('hydra_ros'),
                                   'config/sinks/gvd_places_sinks.yaml'
                               ])),
        DeclareLaunchArgument('active_window_sinks',
                               default_value=PathJoinSubstitution([
                                   FindPackageShare('hydra_ros'),
                                   'config/sinks/active_window_sinks.yaml'
                               ])),
        DeclareLaunchArgument('use_gnn_descriptors', default_value='false'),
        DeclareLaunchArgument('lcd_config_name',
                               default_value='uhumans2.yaml'),
        SetLaunchConfiguration('lcd_config_name', 'uhumans2_gnn.yaml', 
                               condition=IfCondition(LaunchConfiguration('use_gnn_descriptors'))),
        DeclareLaunchArgument('lcd_config_path',
                               default_value=PathJoinSubstitution([
                                   FindPackageShare('hydra_ros'),
                                   'config/lcd/',
                                   LaunchConfiguration('lcd_config_name')
                               ])),
        DeclareLaunchArgument('labelspace_dir',
                               default_value=PathJoinSubstitution([
                                   FindPackageShare('hydra_ros'),
                                   'config/label_spaces/'                            
                               ])),
        DeclareLaunchArgument('labelspace_file', default_value=[
                            LaunchConfiguration('labelspace_name'), '_label_space.yaml']), 
        DeclareLaunchArgument('labelspace_path',
                               default_value=PathJoinSubstitution([
                                   LaunchConfiguration('labelspace_dir'),
                                   LaunchConfiguration('labelspace_file')
                               ])),

        DeclareLaunchArgument('min_glog_level', default_value='0'),
        DeclareLaunchArgument('verbosity', default_value='1'),
        DeclareLaunchArgument('glog_to_file', default_value='false'),
        DeclareLaunchArgument('glog_dir', description='Glog directory', 
                              condition=IfCondition(LaunchConfiguration('glog_to_file'))),
        DeclareLaunchArgument('glog_file_args', default_value=''),
        SetLaunchConfiguration('glog_file_args',
                               PathJoinSubstitution([
                                   TextSubstitution(text='--logtostderr=0 --log_dir='),
                                   LaunchConfiguration('glog_dir')
                               ]),
                               condition=IfCondition(LaunchConfiguration('glog_to_file'))),
        DeclareLaunchArgument('debug', default_value='false'),
        DeclareLaunchArgument('print_missing', default_value='false', 
                              description='Print all missing parameters when parsing configs'),
        DeclareLaunchArgument('launch_prefix', default_value=''),
        SetLaunchConfiguration('launch_prefix', 'gdb -ex run --args', 
                               condition=IfCondition(LaunchConfiguration('debug'))),
        
        # Include pipeline configuration launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('hydra_ros'),
                    'launch', 'pipeline_config.launch.py'
                ])
            )
        ),

        # Group parameters under namespace
        GroupAction(
            actions=[
                # Visualization sinks
                SetParametersFromFile(
                    filename=LaunchConfiguration('active_window_sinks'),
                ),
                SetParametersFromFile(
                    filename=LaunchConfiguration('mesh_segmenter_sinks'),
                ),
                SetParametersFromFile(
                    filename=LaunchConfiguration('gvd_places_sinks'),
                ),
                
                Node(
                    package='hydra_ros',
                    executable='hydra_ros_node',
                    # name=LaunchConfiguration('hydra_ros_ns'),
                    # namespace=LaunchConfiguration('hydra_ros_ns'),
                    output='log' if LaunchConfiguration('glog_to_file') else 'screen',
                    parameters=[
                        ParameterFile(LaunchConfiguration('input_config_path'), allow_substs=True),
                        # ParameterFile(LaunchConfiguration('labelspace_path')),
                        # The labelspace file contains a map that ROS2 can't parse, so can't directly import it
                        {'labelspace_path' : LaunchConfiguration('labelspace_path')},
                        ParameterFile(LaunchConfiguration('hydra_config_path')),
                        ParameterFile(LaunchConfiguration('lcd_config_path')),
                        {'semantic_colormap_file' : LaunchConfiguration('semantic_map_path')},
                        {'semantic_label_remap_filepath' : LaunchConfiguration('semantic_label_remap_filepath')},
                        {'exit_after_clock' : LaunchConfiguration('exit_after_clock')},
                        {'log_path' : LaunchConfiguration('log_path')},
                        {'log_timing_incrementally' : LaunchConfiguration('log_timing_incrementally')},
                        {'disable_timer_output' : LaunchConfiguration('disable_timer_output')},
                        {'timing_disabled' : LaunchConfiguration('timing_disabled')},
                        {'enable_frontend_output' : LaunchConfiguration('enable_frontend_output')},
                        {'frontend.type' : 'GraphBuilder'},
                        {'frontend.enable_places' : LaunchConfiguration('enable_places')},
                        {'frontend.objects.bounding_box_tracker' : LaunchConfiguration('bounding_box_type')},
                        {'frontend.objects.pose_graph_tracker' : LaunchConfiguration('pose_graph_tracker_type')},
                        {'backend.type' : 'BackendModule'},
                        {'backend.optimize_on_lc' : LaunchConfiguration('optimize_on_lc')},
                        # {'backend.zmq_sink.url' : LaunchConfiguration('zmq_send_url')},
                        # {'backend.zmq_sink.send_mesh' : LaunchConfiguration('zmq_send_url')},
                        {'backend.pgmo.add_initial_prior' : LaunchConfiguration('add_initial_prior')},
                        # {'backend.update_functors.zmq_labels.url' : LaunchConfiguration('zmq_recv_url')},
                        {'active_window.type' : 'ReconstructionModule'},
                        {'enable_lcd' : LaunchConfiguration('enable_dsg_lcd')},
                        {'lcd_use_bow_vectors' : LaunchConfiguration('lcd_use_bow_vectors')},
                        {'lcd.use_gnn_descriptors' : LaunchConfiguration('use_gnn_descriptors')},
                        {'lcd.gnn_lcd.object_model_path' : LaunchConfiguration('lcd_gnn_object_model')},
                        {'lcd.gnn_lcd.places_model_path' : LaunchConfiguration('lcd_gnn_places_model')},
                        {'lcd.gnn_lcd.label_embeddings_file' : LaunchConfiguration('lcd_gnn_label_embeddings')},
                        {'lcd.features.type' : LaunchConfiguration('feature_receiver_type')},

                        {'robot_id' : LaunchConfiguration('robot_id')},
                        {'robot_frame' : LaunchConfiguration('robot_frame')},
                        {'odom_frame' : LaunchConfiguration('odom_frame')},
                        {'map_frame' : LaunchConfiguration('map_frame')}
                    ],
                    remappings=[],
                    arguments=[
                        ['--minloglevel=', LaunchConfiguration('min_glog_level')],
                        ['-v=', LaunchConfiguration('verbosity')],
                        LaunchConfiguration('glog_file_args')
                    ],
                    # env={'TERM': 'xterm-256color'},
                )
                
            ]
        ),


        
    ])

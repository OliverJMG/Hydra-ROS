cmake_minimum_required(VERSION 3.16)
project(hydra_visualizer)

add_compile_options(-Wall -Wextra)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(config_utilities REQUIRED)
find_package(gflags REQUIRED)
find_package(spark_dsg REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hydra_msgs REQUIRED)
find_package(kimera_pgmo_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)


# generate_dynamic_reconfigure_options(
#   cfg/DynamicLayerVisualizer.cfg cfg/LayerVisualizer.cfg cfg/Visualizer.cfg
# )

add_library(
  ${PROJECT_NAME}
  src/dsg_visualizer.cpp
  src/scene_graph_renderer.cpp
  src/color/colormap_utilities.cpp
  src/color/graph_color_adaptors.cpp
  src/color/mesh_color_adaptor.cpp
  src/io/graph_file_wrapper.cpp
  src/io/graph_ros_wrapper.cpp
  src/io/graph_zmq_wrapper.cpp
  src/groundtruth/region_publisher.cpp
  src/groundtruth/bounding_box_publisher.cpp
  src/plugins/basis_point_plugin.cpp
  src/plugins/khronos_object_plugin.cpp
  src/plugins/mesh_plugin.cpp
  src/plugins/footprint_plugin.cpp
  src/utils/configs.cpp
  src/utils/config_manager.cpp
  src/utils/ear_clipping.cpp
  src/utils/label_adaptors.cpp
  src/utils/marker_group_pub.cpp
  src/utils/marker_tracker.cpp
  src/utils/node_filter.cpp
  src/utils/polygon_utilities.cpp
  src/utils/visualizer_utilities.cpp
  src/utils/visualizer_types.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_link_libraries(
  ${PROJECT_NAME} PUBLIC  config_utilities::config_utilities
                         spark_dsg::spark_dsg
)
ament_target_dependencies(${PROJECT_NAME} PUBLIC
  hydra_msgs
  kimera_pgmo_msgs
  rclcpp
  std_msgs
  std_srvs
  tf2_eigen
  tf2_ros
  visualization_msgs
)

add_executable(hydra_visualizer_node app/hydra_visualizer_node.cpp)
target_link_libraries(hydra_visualizer_node ${PROJECT_NAME} gflags)

# if(BUILD_TESTING)
#   ament_add_gtest(test_${PROJECT_NAME} tests/main.cpp tests/test_ear_clipping.cpp)
#   target_link_libraries(
#     test_${PROJECT_NAME}
#     ${PROJECT_NAME}
#     gflags
#   )
# endif()

install(
  TARGETS ${PROJECT_NAME} 
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS hydra_visualizer_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ 
        DESTINATION include/)

install(DIRECTORY launch config
        DESTINATION share/${PROJECT_NAME}/)

ament_package()
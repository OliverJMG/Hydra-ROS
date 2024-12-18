#include "hydra_visualizer/utils/configs.h"


namespace hydra::visualizer {

void declare_config(VisualizerConfig& config) {
    config::name("VisualizerConfig");
    config::field(config.layer_z_step, "layer_z_step");
}

void declare_config(LayerVisualizerConfig& config) {
    config::name("LayerVisualizerConfig");
    config::field(config.z_offset_scale, "z_offset_scale");
    config::field(config.visualize, "visualize");

    config::field(config.marker_scale, "marker_scale");
    config::field(config.marker_color_mode, "marker_color_mode");
    config::field(config.marker_alpha, "marker_alpha");
    config::field(config.use_sphere_marker, "use_sphere_marker");

    config::field(config.use_label, "use_label");
    config::field(config.collapse_label, "collapse_label");
    config::field(config.label_mode, "label_mode");
    config::field(config.label_height, "label_height");
    config::field(config.label_scale, "label_scale");
    config::field(config.add_label_jitter, "add_label_jitter");
    config::field(config.label_jitter_scale, "label_jitter_scale");

    config::field(config.use_bounding_box, "use_bounding_box");
    config::field(config.collapse_bounding_box, "collapse_bounding_box");
    config::field(config.bounding_box_scale, "bounding_box_scale");
    config::field(config.bounding_box_edge_scale, "bounding_box_edge_scale");
    config::field(config.bounding_box_alpha, "bounding_box_alpha");
    config::field(config.mesh_edge_break_ratio, "mesh_edge_break_ratio");

    config::field(config.draw_boundaries, "draw_boundaries");
    config::field(config.collapse_boundary, "collapse_boundary");
    config::field(config.boundary_wireframe_scale, "boundary_wireframe_scale");
    config::field(config.boundary_use_node_color, "boundary_use_node_color");
    config::field(config.boundary_alpha, "boundary_alpha");
    config::field(config.draw_boundary_ellipse, "draw_boundary_ellipse");
    config::field(config.boundary_ellipse_alpha, "boundary_ellipse_alpha");
    config::field(config.draw_frontier_ellipse, "draw_frontier_ellipse");
    config::field(config.draw_mesh_edges, "draw_mesh_edges");

    config::field(config.edge_scale, "edge_scale");
    config::field(config.edge_alpha, "edge_alpha");
    config::field(config.edge_use_color, "edge_use_color");
    config::field(config.interlayer_edge_use_source, "interlayer_edge_use_source");
    config::field(config.interlayer_edge_scale, "interlayer_edge_scale");
    config::field(config.interlayer_edge_alpha, "interlayer_edge_alpha");
    config::field(config.interlayer_edge_use_color, "interlayer_edge_use_color");
    config::field(config.interlayer_edge_insertion_skip, "interlayer_edge_insertion_skip");
}

void declare_config(DynamicLayerVisualizerConfig& config) {
    config::name("DynamicLayerVisualizerConfig");
    config::field(config.z_offset_scale, "z_offset_scale");
    config::field(config.visualize, "visualize");
    config::field(config.node_scale, "node_scale");
    config::field(config.node_alpha, "node_alpha");
    config::field(config.node_use_sphere, "node_use_sphere");
    config::field(config.node_color_mode, "node_color_mode");
    config::field(config.edge_scale, "edge_scale");
    config::field(config.edge_alpha, "edge_alpha");
    config::field(config.label_mode, "label_mode");
    config::field(config.label_height, "label_height");
    config::field(config.label_scale, "label_scale");
    config::field(config.interlayer_edge_insertion_skip, "interlayer_edge_insertion_skip");
    config::field(config.visualize_interlayer_edges, "visualize_interlayer_edges");
}


}
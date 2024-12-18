#pragma once

#include <string>
#include <config_utilities/config.h>

// TODO(Oliver): Implement checks and enum types
namespace hydra::visualizer {

struct VisualizerConfig {
    // Size of the distance between layers
    double layer_z_step = 5.0;
    // Whether or not to show the layers independently
    bool collapse_layers = false;
};

void declare_config(VisualizerConfig& config);

struct LayerVisualizerConfig {
    // Number of steps of offset to apply
    double z_offset_scale = 0.0; // Range: [-5.0, 10.0]
    // Whether to visualize the layer
    bool visualize = false;

    // NODES //
    // Size of the centroid marker
    double marker_scale = 0.1;
    // Node color adaptor type
    std::string marker_color_mode = "NodeColorAdaptor";
    // Alpha of the centroid marker
    double marker_alpha = 1.0;
    // Use sphere markers (instead of cubes)
    bool use_sphere_marker = false;

    // LABELS //
    // Whether to use labels or not
    bool use_label = false;
    // Whether to collapse all labels into one at mesh level
    bool collapse_label = false;
    // Type of label adaptor 
    std::string label_mode = "IdLabelAdaptor";
    // Height of the label above the node
    double label_height = 1.0; // Range: [0.0, 5.0]
    // Scaling factor for the label size
    double label_scale = 0.5; // Range: [0.05, 5.0]
    // Whether to add random noise to the label height
    bool add_label_jitter = false;
    // Scale of the jitter effect for labels
    double label_jitter_scale = 0.2; // Range: [0.05, 5.0]

    // BOUNDING BOXES //
    // Whether to display bounding boxes
    bool use_bounding_box = false;
    // Whether to collapse bounding boxes at ground level
    bool collapse_bounding_box = false;
    // Scaling factor for bounding box wireframe
    double bounding_box_scale = 0.1; // Range: [0.001, 1.0]
    // Scaling factor for bounding box edge connections
    double bounding_box_edge_scale = 0.01; // Range: [0.001, 1.0]
    // Transparency of the bounding box
    double bounding_box_alpha = 0.5; // Range: [0.0, 1.0]
    // Ratio at which edges break into smaller edges
    double mesh_edge_break_ratio = 0.5; // Range: [0.0, 1.0]

    // POLYGONS //
    // Whether to display polygon boundaries
    bool draw_boundaries = false;
    // Whether to collapse boundaries at mesh level
    bool collapse_boundary = false;
    // Scale of the boundary wireframe
    double boundary_wireframe_scale = 0.1; // Range: [0.001, 1.0]
    // Whether to use node semantic colors for boundaries
    bool boundary_use_node_color = true;
    // Transparency of the boundary
    double boundary_alpha = 0.5; // Range: [0.0, 1.0]
    // Whether to display the minimum bounding ellipse
    bool draw_boundary_ellipse = false;
    // Transparency of the boundary ellipse
    double boundary_ellipse_alpha = 0.5; // Range: [0.0, 1.0]
    // Whether to display the frontier ellipse approximation
    bool draw_frontier_ellipse = false;
    // Whether to draw mesh edges
    bool draw_mesh_edges = false;

    // EDGES //
    // Scale of intralayer edges
    double edge_scale = 0.03; // Range: [0.001, 1.0]
    // Transparency of intralayer edges
    double edge_alpha = 1.0; // Range: [0.0, 1.0]
    // Whether to use color for intralayer edges
    bool edge_use_color = true;
    // Whether to use the source layer for interlayer edges
    bool interlayer_edge_use_source = true;
    // Scale of interlayer edges
    double interlayer_edge_scale = 0.03; // Range: [0.001, 1.0]
    // Transparency of interlayer edges
    double interlayer_edge_alpha = 1.0; // Range: [0.0, 1.0]
    // Whether to use color for interlayer edges
    bool interlayer_edge_use_color = true;
    // Number of edges to skip when drawing interlayer edges
    int interlayer_edge_insertion_skip = 0; // Range: [0, 1000]
};

void declare_config(LayerVisualizerConfig& config);

struct DynamicLayerVisualizerConfig {
    // Number of steps of offset to apply
    double z_offset_scale = 0.0; // Range: [-5.0, 10.0]
    // Whether to visualize the layer
    bool visualize = true;
    // Size of the node markers
    double node_scale = 0.1; // Range: [0.01, 2.0]
    // Transparency of the node markers
    double node_alpha = 1.0; // Range: [0.0, 1.0]
    // Whether to use sphere markers for nodes instead of cubes
    bool node_use_sphere = false;
    // Node color adaptor type (e.g., "PrefixColorAdaptor")
    std::string node_color_mode = "PrefixColorAdaptor";
    // Size of the edges within a layer
    double edge_scale = 0.03; // Range: [0.001, 1.0]
    // Transparency of the edges within a layer
    double edge_alpha = 1.0; // Range: [0.0, 1.0]
    // Label adaptor type (e.g., "NameLabelAdaptor")
    std::string label_mode = "NameLabelAdaptor";
    // Height of the text label above the latest node in the layer
    double label_height = 1.0; // Range: [0.0, 5.0]
    // Scale of the text label above the latest node in the layer
    double label_scale = 0.5; // Range: [0.05, 5.0]
    // Number of edges to skip when drawing interlayer edges
    int interlayer_edge_insertion_skip = 0; // Range: [0, 1000]
    // Whether to visualize interlayer edges
    bool visualize_interlayer_edges = false;
};

void declare_config(DynamicLayerVisualizerConfig& config);

}

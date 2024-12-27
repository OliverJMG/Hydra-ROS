#pragma once

namespace hydra {

struct GvdVisualizerConfig {
    // Whether to show voxel block outlines
    bool show_block_outlines = true;
    // Scale for block outlines
    double block_outline_scale = 0.02; // Range: [0.0, 1.0]
    // Transparency of the GVD
    double gvd_alpha = 0.6; // Range: [0.0, 1.0]
    // Minimum transparency of the GVD
    double gvd_min_alpha = 0.6; // Range: [0.0, 1.0]
    // Minimum distance for the distance colormap
    double gvd_min_distance = 0.2; // Range: [-5.0, 5.0]
    // Maximum distance for the distance colormap
    double gvd_max_distance = 3.0; // Range: [-5.0, 5.0]
    // Basis threshold for GVD inclusion
    int basis_threshold = 2; // Range: [1, 26]
    // Minimum number of bases for colormap
    int min_num_basis = 1; // Range: [1, 26]
    // Maximum number of bases for colormap
    int max_num_basis = 26; // Range: [1, 26]
    // Visualization mode (e.g., 0: Basic, 1: Wireframe, 2: Graph)
    enum class Mode {DEFAULT, DISTANCE, BASIS_POINTS} gvd_mode = Mode::DEFAULT;
    // Scale for wireframe visualization
    double gvd_graph_scale = 0.005; // Range: [0.0, 1.0]
    // Transparency for free space spheres
    double freespace_sphere_alpha = 0.15; // Range: [0.0, 1.0]
    // Transparency of the ESDF
    double esdf_alpha = 0.6; // Range: [0.0, 1.0]
    // Height of the ESDF slice
    double slice_height = 0.0; // Range: [-10.0, 10.0]
    // Distance colormap scale
    double esdf_distance = 2.0; // Range: [0.0, 5.0]
};

void declare_config(GvdVisualizerConfig& config);

}

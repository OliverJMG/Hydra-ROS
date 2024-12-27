#include "hydra_ros/frontend/gvd_visualizer_config.h"

#include <config_utilities/config.h>
#include <config_utilities/types/enum.h>

namespace hydra {

void declare_config(GvdVisualizerConfig& config) {
    config::name("GvdVisualizerConfig");
    config::field(config.show_block_outlines, "show_block_outlines");
    config::field(config.block_outline_scale, "block_outline_scale");
    config::field(config.gvd_alpha, "gvd_alpha");
    config::field(config.gvd_min_alpha, "gvd_min_alpha");
    config::field(config.gvd_min_distance, "gvd_min_distance");
    config::field(config.gvd_max_distance, "gvd_max_distance");
    config::field(config.basis_threshold, "basis_threshold");
    config::field(config.min_num_basis, "min_num_basis");
    config::field(config.max_num_basis, "max_num_basis");
    config::enum_field(config.gvd_mode, "gvd_mode", {"DEFAULT", "DISTANCE", "BASIS_POINTS"});
    config::field(config.gvd_graph_scale, "gvd_graph_scale");
    config::field(config.freespace_sphere_alpha, "freespace_sphere_alpha");
    config::field(config.esdf_alpha, "esdf_alpha");
    config::field(config.slice_height, "slice_height");
    config::field(config.esdf_distance, "esdf_distance");
}

}
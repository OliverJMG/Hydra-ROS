/* -----------------------------------------------------------------------------
 * Copyright 2022 Massachusetts Institute of Technology.
 * All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Research was sponsored by the United States Air Force Research Laboratory and
 * the United States Air Force Artificial Intelligence Accelerator and was
 * accomplished under Cooperative Agreement Number FA8750-19-2-1000. The views
 * and conclusions contained in this document are those of the authors and should
 * not be interpreted as representing the official policies, either expressed or
 * implied, of the United States Air Force or the U.S. Government. The U.S.
 * Government is authorized to reproduce and distribute reprints for Government
 * purposes notwithstanding any copyright notation herein.
 * -------------------------------------------------------------------------- */
#pragma once
#include <hydra/places/compression_graph_extractor.h>
#include <hydra/places/gvd_graph.h>
#include <hydra/places/gvd_voxel.h>
#include <hydra_visualizer/color/colormap_utilities.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "hydra_ros/frontend/gvd_visualizer_config.h"

namespace hydra {

using CompressedNodeMap = std::unordered_map<uint64_t, places::CompressedNode>;
using ClusterRemapping = std::unordered_map<uint64_t, uint64_t>;
using hydra::GvdVisualizerConfig;

visualization_msgs::msg::Marker drawEsdf(const GvdVisualizerConfig& config,
                                    const visualizer::RangeColormap& colormap,
                                    const Eigen::Isometry3d& pose,
                                    const places::GvdLayer& layer,
                                    const std::string& ns);

visualization_msgs::msg::Marker drawGvd(const GvdVisualizerConfig& config,
                                   const visualizer::RangeColormap& colormap,
                                   const places::GvdLayer& layer,
                                   const std::string& ns);

visualization_msgs::msg::Marker drawGvdSurface(const GvdVisualizerConfig& config,
                                          const visualizer::RangeColormap& colormap,
                                          const places::GvdLayer& layer,
                                          const std::string& ns);

visualization_msgs::msg::Marker drawGvdError(const GvdVisualizerConfig& config,
                                        const visualizer::RangeColormap& colormap,
                                        const places::GvdLayer& lhs,
                                        const places::GvdLayer& rhs,
                                        double threshold);

visualization_msgs::msg::MarkerArray drawGvdGraph(const places::GvdGraph& graph,
                                             const GvdVisualizerConfig& config,
                                             const visualizer::RangeColormap& cmap,
                                             const std::string& ns,
                                             size_t marker_id = 0);

visualization_msgs::msg::MarkerArray drawGvdClusters(
    const places::GvdGraph& graph,
    const CompressedNodeMap& clusters,
    const ClusterRemapping& remapping,
    const GvdVisualizerConfig& config,
    const std::string& ns,
    const visualizer::DiscreteColormap& cmap = {},
    size_t marker_id = 0);

visualization_msgs::msg::MarkerArray drawPlaceFreespace(const std_msgs::msg::Header& header,
                                                   const SceneGraphLayer& layer,
                                                   const std::string& ns,
                                                   const spark_dsg::Color& color);

}  // namespace hydra

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

#include <config_utilities/virtual_config.h>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>

#include <fstream>

#include "hydra_visualizer/io/graph_file_wrapper.h"
#include "hydra_visualizer/io/graph_ros_wrapper.h"
#include "hydra_visualizer/io/graph_zmq_wrapper.h"
#include "hydra_visualizer/plugins/basis_point_plugin.h"
#include "hydra_visualizer/plugins/footprint_plugin.h"
#include "hydra_visualizer/plugins/khronos_object_plugin.h"
#include "hydra_visualizer/plugins/mesh_plugin.h"
#include "hydra_visualizer/scene_graph_renderer.h"

namespace hydra {

class DsgVisualizer : public rclcpp::Node {
 public:
  struct Config {
    std::string ns = "~";
    double loop_period_s = 0.1;
    std::string visualizer_frame = "map";
    config::VirtualConfig<GraphWrapper> graph;
    // Specify additional plugins that should be loaded <name, config>
    std::map<std::string, config::VirtualConfig<VisualizerPlugin>> plugins;
  } config;

  //! Construct the visualizer
  explicit DsgVisualizer();

  ~DsgVisualizer() = default;

  //! Set up the config, node must be initialized first
  void configure();

  //! Loop and redraw when changes occur
  void start();

  //! Delete all currently published visualization artifacts and remake graph
  void reset();

  //! Add a new graph plugin
  void addPlugin(VisualizerPlugin::Ptr plugin);

  //! Delete all current plugins
  void clearPlugins();

 private:
  void spinOnce(bool force = false);

  void redraw(std_srvs::srv::Empty::Request::SharedPtr, std_srvs::srv::Empty::Response::SharedPtr);
  void reset_cb(std_srvs::srv::Empty::Request::SharedPtr, std_srvs::srv::Empty::Response::SharedPtr);

  rclcpp::TimerBase::SharedPtr loop_timer_;

  GraphWrapper::Ptr graph_;
  SceneGraphRenderer::Ptr renderer_;
  std::vector<VisualizerPlugin::Ptr> plugins_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr redraw_service_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_service_;
};

void declare_config(DsgVisualizer::Config& config);

}  // namespace hydra

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

#include "hydra_visualizer/dsg_visualizer.h"

#include <config_utilities/parsing/ros2.h>
#include <config_utilities/config.h>
#include <config_utilities/printing.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>

using std::placeholders::_1;
using std::placeholders::_2;

namespace hydra {

void declare_config(DsgVisualizer::Config& config) {
  using namespace config;
  name("HydraVisualizerConfig");
  field(config.ns, "ns");
  field(config.loop_period_s, "loop_period_s", "s");
  field(config.visualizer_frame, "visualizer_frame");
  field(config.graph, "graph");
  field(config.plugins, "plugins");
  checkCondition(!config.visualizer_frame.empty(), "visualizer_frame");
}

DsgVisualizer::DsgVisualizer()
    : Node("hydra_dsg_visualizer", rclcpp::NodeOptions().allow_undeclared_parameters(true)
                        .automatically_declare_parameters_from_overrides(true)) {}

void DsgVisualizer::configure() {
  config = config::fromRos<hydra::DsgVisualizer::Config>(this->get_node_parameters_interface());
  // config::checkValid<hydra::DsgVisualizer::Config>(config);
  RCLCPP_INFO_STREAM(get_logger(), config::toString(config));
  renderer_ = std::make_shared<SceneGraphRenderer>(this->shared_from_this());
  
  for (auto&& [name, plugin] : config.plugins) {
    plugins_.push_back(plugin.create(this->shared_from_this(), name));
  }
  graph_ = config.graph.create();

  redraw_service_ = this->create_service<std_srvs::srv::Empty>("redraw", 
          std::bind(&DsgVisualizer::redraw, this, _1, _2));
  reset_service_ = this->create_service<std_srvs::srv::Empty>("reset", 
          std::bind(&DsgVisualizer::reset_cb, this, _1, _2));
}

void DsgVisualizer::start() {
  loop_timer_ = this->create_wall_timer(std::chrono::duration<double>(config.loop_period_s),
                                    [this]() -> void { spinOnce(); });
}

void DsgVisualizer::reset() {
  std_msgs::msg::Header header;
  header.stamp = this->get_clock()->now();
  header.frame_id = config.visualizer_frame;

  renderer_->reset(header);
  for (const auto& plugin : plugins_) {
    if (plugin) {
      plugin->reset(header);
    }
  }

  graph_ = config.graph.create();
}

void DsgVisualizer::addPlugin(VisualizerPlugin::Ptr plugin) {
  plugins_.push_back(std::move(plugin));
}

void DsgVisualizer::clearPlugins() { plugins_.clear(); }

void DsgVisualizer::spinOnce(bool force) {
  if (!graph_) {
    return;
  }

  bool has_change = false;
  has_change = graph_->hasChange();
  has_change |= renderer_->hasChange();
  for (const auto& plugin : plugins_) {
    if (plugin) {
      has_change |= plugin->hasChange();
    }
  }

  if (!has_change && !force) {
    return;
  }

  const auto stamped_graph = graph_->get();
  if (!stamped_graph) {
    return;
  }

  std_msgs::msg::Header header;
  header.frame_id = config.visualizer_frame;
  header.stamp = stamped_graph.timestamp.value_or(this->get_clock()->now());

  renderer_->draw(header, *stamped_graph.graph);
  for (const auto& plugin : plugins_) {
    if (plugin) {
      plugin->draw(header, *stamped_graph.graph);
    }
  }

  graph_->clearChangeFlag();
  renderer_->clearChangeFlag();
  for (auto& plugin : plugins_) {
    if (plugin) {
      plugin->clearChangeFlag();
    }
  }
}

void DsgVisualizer::redraw(std_srvs::srv::Empty::Request::SharedPtr, 
        std_srvs::srv::Empty::Response::SharedPtr) {
  spinOnce(true);
}

void DsgVisualizer::reset_cb(std_srvs::srv::Empty::Request::SharedPtr, 
        std_srvs::srv::Empty::Response::SharedPtr) {
  reset();
}

}  // namespace hydra

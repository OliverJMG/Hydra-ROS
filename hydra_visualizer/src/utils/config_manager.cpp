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
#include "hydra_visualizer/utils/config_manager.h"

#include <config_utilities/parsing/ros2.h>
#include <config_utilities/parsing/yaml.h>
// #include <dynamic_reconfigure/server.h>
#include <glog/logging.h>

namespace hydra::visualizer {

using hydra::visualizer::DynamicLayerVisualizerConfig;
using hydra::visualizer::LayerVisualizerConfig;
using hydra::visualizer::VisualizerConfig;
using spark_dsg::Color;
using spark_dsg::DynamicSceneGraph;
using spark_dsg::LayerId;
using spark_dsg::SceneGraphNode;

const std::string& getColorMode(const LayerVisualizerConfig& config) {
  return config.marker_color_mode;
}

const std::string& getColorMode(const DynamicLayerVisualizerConfig& config) {
  return config.node_color_mode;
}

ColorManager::ColorManager(const rclcpp::Node::SharedPtr node, spark_dsg::LayerId layer)
    : has_change_(false), node_(node), layer_(layer) {
  // NOTE(nathan) this is ugly but probably the easiest way to parse the current
  // settings from ros
  std::stringstream ss;
  ss << config::internal::rosToYaml(node_);
  curr_contents_ = ss.str();
  sub_ = node_->create_subscription<std_msgs::msg::String>("", 1, 
          std::bind(&ColorManager::callback, this, std::placeholders::_1));
}

ColorManager::ColorFunc ColorManager::get(const DynamicSceneGraph& graph) const {
  if (!adaptor_) {
    return DefaultNodeColorFunction();
  }

  adaptor_->setGraph(graph, layer_);
  return [this, &graph](const SceneGraphNode& node) {
    return adaptor_->getColor(graph, node);
  };
}

void ColorManager::set(const std::string& mode) {
  if (mode_ != mode) {
    mode_ = mode;
    setAdaptor();
  }
}

void ColorManager::setAdaptor() {
  has_change_ = true;
  try {
    auto node = YAML::Load(curr_contents_);
    node["type"] = mode_;
    VLOG(5) << "Attempting creation from " << node;
    adaptor_ = config::createFromYaml<GraphColorAdaptor>(node);
  } catch (const std::exception& e) {
    LOG(ERROR) << "Failed to parse adaptor settings from: " << curr_contents_;
    adaptor_.reset();
  }
}

bool ColorManager::hasChange() const { return has_change_; }

void ColorManager::clearChangeFlag() { has_change_ = false; }

void ColorManager::callback(const std_msgs::msg::String::ConstSharedPtr& msg) {
  curr_contents_ = msg->data;
  setAdaptor();
}

LabelManager::LabelManager(const rclcpp::Node::SharedPtr node)
    : has_change_(false), node_(node) {
  // NOTE(nathan) this is ugly but probably the easiest way to parse the current
  // settings from ros
  std::stringstream ss;
  ss << config::internal::rosToYaml(node_);
  curr_contents_ = ss.str();
  sub_ = node_->create_subscription<std_msgs::msg::String>("", 1, 
          std::bind(&LabelManager::callback, this, std::placeholders::_1));
}

LabelManager::LabelFunc LabelManager::get() const {
  if (!adaptor_) {
    return DefaultNodeLabelFunction();
  }

  return [this](const SceneGraphNode& node) { return adaptor_->getLabel(node); };
}

void LabelManager::set(const std::string& mode) {
  if (mode_ != mode) {
    mode_ = mode;
    setAdaptor();
  }
}

void LabelManager::setAdaptor() {
  has_change_ = true;
  try {
    auto node = YAML::Load(curr_contents_);
    node["type"] = mode_;
    adaptor_ = config::createFromYaml<GraphLabelAdaptor>(node);
    VLOG(5) << "Attempting creation from " << node << " success: (" << std::boolalpha
            << (adaptor_ != nullptr) << ")";
  } catch (const std::exception& e) {
    LOG(ERROR) << "Failed to parse adaptor settings from: " << curr_contents_;
    adaptor_.reset();
  }
}

bool LabelManager::hasChange() const { return has_change_; }

void LabelManager::clearChangeFlag() { has_change_ = false; }

void LabelManager::callback(const std_msgs::msg::String::ConstSharedPtr& msg) {
  curr_contents_ = msg->data;
  setAdaptor();
}

ConfigManager::ConfigManager() {}

ConfigManager::~ConfigManager() {
  visualizer_config_.reset();
  layers_.clear();
  dynamic_layers_.clear();
}

ConfigManager& ConfigManager::instance() {
  if (!s_instance_) {
    s_instance_.reset(new ConfigManager());
  }

  return *s_instance_;
}

void ConfigManager::init(const rclcpp::Node::SharedPtr node) {
  instance().node_ = node;
  reset();
}

void ConfigManager::reset() {
  auto& curr = instance();
  curr.visualizer_config_.reset();
  curr.layers_.clear();
  curr.dynamic_layers_.clear();
}

void ConfigManager::reset(const DynamicSceneGraph& graph) {
  reset();

  // initialize the global config
  instance().getVisualizerConfig();

  for (const auto layer : graph.layer_ids) {
    // this initializes the underlying config even if we don't use the return
    instance().getLayerConfig(layer);
  }

  for (const auto& [layer_id, layers] : graph.dynamicLayers()) {
    // this initializes the underlying config even if we don't use the return
    instance().getDynamicLayerConfig(layer_id);
  }
}

bool ConfigManager::hasChange() const {
  bool has_changed = visualizer_config_ ? visualizer_config_->hasChange() : false;
  for (const auto& id_config_pair : layers_) {
    has_changed |= id_config_pair.second.hasChange();
  }

  for (const auto& id_config_pair : dynamic_layers_) {
    has_changed |= id_config_pair.second.hasChange();
  }

  return has_changed;
}

void ConfigManager::clearChangeFlags() {
  if (visualizer_config_) {
    visualizer_config_->clearChangeFlag();
  }

  for (auto& id_config_pair : layers_) {
    id_config_pair.second.clearChangeFlag();
  }

  for (auto& id_config_pair : dynamic_layers_) {
    id_config_pair.second.clearChangeFlag();
  }
}

const VisualizerConfig& ConfigManager::getVisualizerConfig() const {
  if (!visualizer_config_) {
    visualizer_config_ =
        std::make_shared<ConfigWrapper<VisualizerConfig>>(
          "/workspaces/ros2_ws/src/hydra_ros/hydra_visualizer/config/visualizer_defaults.yaml",
          "");
  }

  return visualizer_config_->get();
}

const StaticLayerConfig& ConfigManager::getLayerConfig(LayerId layer) const {
  auto iter = layers_.find(layer);
  if (iter == layers_.end()) {
    // const auto ns = "config/layer" + std::to_string(layer);
    iter = layers_.emplace(layer, StaticLayerConfig(
        "/workspaces/ros2_ws/src/hydra_ros/hydra_visualizer/config/layer_defaults.yaml", 
        "", node_, layer)).first;
  }

  return iter->second;
}

const DynamicLayerConfig& ConfigManager::getDynamicLayerConfig(LayerId layer) const {
  auto iter = dynamic_layers_.find(layer);
  if (iter == dynamic_layers_.end()) {
    // const std::string ns = "config/dynamic_layer/layer" + std::to_string(layer);
    iter = dynamic_layers_.emplace(layer, DynamicLayerConfig(
        "/workspaces/ros2_ws/src/hydra_ros/hydra_visualizer/config/dynamic_layer_defaults.yaml",
        "", node_, layer)).first;
  }

  return iter->second;
}

}  // namespace hydra::visualizer

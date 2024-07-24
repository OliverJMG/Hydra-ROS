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
#include <ros/ros.h>
#include <spark_dsg/dynamic_scene_graph.h>
#include <visualization_msgs/MarkerArray.h>

#include <string>
#include <vector>

#include "hydra_ros/visualizer/marker_tracker.h"
#include "hydra_ros/visualizer/visualizer_types.h"

namespace hydra {

class DsgVisualizerPlugin;

using visualization_msgs::Marker;
using visualization_msgs::MarkerArray;

class DynamicSceneGraphVisualizer {
 public:
  struct Config {
    std::string visualizer_frame = "map";
  } const config;

  explicit DynamicSceneGraphVisualizer(const ros::NodeHandle& nh);

  virtual ~DynamicSceneGraphVisualizer() = default;

  void reset();

  bool redraw();

  void start(bool periodic_redraw = false);

  void setGraph(const spark_dsg::DynamicSceneGraph::Ptr& scene_graph,
                bool need_reset = true);

  bool graphIsSet() const;

  spark_dsg::DynamicSceneGraph::Ptr getGraph();

  void addPlugin(const std::shared_ptr<DsgVisualizerPlugin>& plugin);

  void clearPlugins();

  void setNeedRedraw();

  void setGraphUpdated();

 protected:
  void displayLoop(const ros::WallTimerEvent&);

  virtual void redrawImpl(const std_msgs::Header& header);

  virtual void drawLayer(const std_msgs::Header& header,
                         const visualizer::StaticLayerInfo& info,
                         const spark_dsg::SceneGraphLayer& layer,
                         MarkerArray& msg);

  virtual void drawDynamicLayer(const std_msgs::Header& header,
                                const visualizer::DynamicLayerInfo& info,
                                const spark_dsg::DynamicSceneGraphLayer& layer,
                                MarkerArray& msg);

 protected:
  ros::NodeHandle nh_;
  ros::WallTimer visualizer_loop_timer_;

  bool need_redraw_;
  bool periodic_redraw_;
  spark_dsg::DynamicSceneGraph::Ptr graph_;

  ros::Publisher pub_;
  MarkerTracker tracker_;
  std::list<std::shared_ptr<DsgVisualizerPlugin>> plugins_;
};

void declare_config(DynamicSceneGraphVisualizer::Config& config);

}  // namespace hydra

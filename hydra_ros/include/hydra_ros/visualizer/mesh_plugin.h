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
#include <config_utilities/factory.h>
#include <std_srvs/SetBool.h>

#include "hydra_ros/visualizer/dsg_visualizer_plugin.h"

namespace hydra {

class SemanticColorMap;
struct MeshColoring;

class MeshPlugin : public DsgVisualizerPlugin {
 public:
  using Labels = std::vector<uint32_t>;
  using LabelsPtr = std::shared_ptr<Labels>;

  struct Config {
    std::string label_colormap = "";
    bool color_by_label = false;
  } const config;

  MeshPlugin(const Config& config, const ros::NodeHandle& nh, const std::string& name);

  virtual ~MeshPlugin();

  void draw(const std_msgs::Header& header,
            const spark_dsg::DynamicSceneGraph& graph) override;

  void reset(const std_msgs::Header& header) override;

  bool hasChange() const override;

  void clearChangeFlag() override;

 protected:
  bool handleService(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);

  std::string getMsgNamespace() const;

  bool color_by_label_ = false;
  bool need_redraw_ = true;
  ros::Publisher mesh_pub_;
  ros::ServiceServer toggle_service_;
  std::unique_ptr<SemanticColorMap> colormap_;
  std::shared_ptr<const MeshColoring> mesh_coloring_;

  inline static const auto registration_ =
      config::RegistrationWithConfig<DsgVisualizerPlugin,
                                     MeshPlugin,
                                     MeshPlugin::Config,
                                     ros::NodeHandle,
                                     std::string>("MeshPlugin");
};

void declare_config(MeshPlugin::Config& config);

}  // namespace hydra

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
#include <hydra/frontend/gvd_place_extractor.h>

#include "hydra_ros/ColormapConfig.h"
#include "hydra_ros/GvdVisualizerConfig.h"
#include "hydra_ros/LayerVisualizerConfig.h"
#include "hydra_ros/visualizer/config_wrapper.h"
#include "hydra_ros/visualizer/marker_group_pub.h"

namespace hydra {
namespace places {
class GraphExtractorInterface;
}  // namespace places

class PlacesVisualizer : public GvdPlaceExtractor::Sink {
 public:
  struct Config {
    std::string ns = "~places";
  } const config;

  explicit PlacesVisualizer(const Config& config);

  virtual ~PlacesVisualizer() = default;

  std::string printInfo() const override;

  void call(uint64_t timestamp_ns,
            const Eigen::Isometry3f& world_T_body,
            const places::GvdLayer& gvd,
            const places::GraphExtractorInterface* extractor) const override;

 private:
  void visualizeGvd(const std_msgs::Header& header, const places::GvdLayer& gvd) const;

  void visualizeExtractor(const std_msgs::Header& header,
                          const places::GraphExtractorInterface& extractor) const;

  void visualizeGraph(const std_msgs::Header& header,
                      const SceneGraphLayer& graph) const;

 protected:
  ros::NodeHandle nh_;
  MarkerGroupPub pubs_;
  visualizer::ConfigWrapper<hydra_ros::ColormapConfig> colormap_;
  visualizer::ConfigWrapper<hydra_ros::GvdVisualizerConfig> gvd_config_;
  visualizer::ConfigWrapper<hydra_ros::LayerVisualizerConfig> layer_config_;

 private:
  inline static const auto registration_ =
      config::RegistrationWithConfig<GvdPlaceExtractor::Sink, PlacesVisualizer, Config>(
          "PlacesVisualizer");
};

void declare_config(PlacesVisualizer::Config& config);

}  // namespace hydra

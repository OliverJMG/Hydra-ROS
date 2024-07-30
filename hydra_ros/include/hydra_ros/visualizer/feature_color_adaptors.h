#pragma once
#include <hydra/openset/embedding_distances.h>
#include <ros/ros.h>

#include "hydra_ros/openset/ros_embedding_group.h"
#include "hydra_ros/visualizer/graph_color_adaptors.h"

namespace hydra {

class FeatureScoreColor : public GraphColorAdaptor {
 public:
  struct Config {
    std::string ns = "~";
    float min_score = 0.0f;
    float max_score = 1.0f;
    bool use_fixed_range = false;
    config::VirtualConfig<EmbeddingDistance> metric{CosineDistance::Config()};
  } const config;

  explicit FeatureScoreColor(const Config& config);
  ~FeatureScoreColor();
  void setGraph(const spark_dsg::DynamicSceneGraph& graph,
                spark_dsg::LayerId layer) override;
  spark_dsg::Color getColor(const spark_dsg::DynamicSceneGraph& graph,
                            const spark_dsg::SceneGraphNode& node) const override;

  void setFeature(const Eigen::VectorXf& feature);

 private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  struct FeatureTrampoline;
  std::unique_ptr<FeatureTrampoline> trampoline_;

  struct ScoreRange {
    float min = 0.0f;
    float max = 1.0f;
  } range_;

  bool has_change_;
  bool has_feature_;
  Eigen::VectorXf feature_;
  std::unordered_map<spark_dsg::NodeId, float> values_;
  std::unique_ptr<EmbeddingDistance> metric_;

  inline static const auto registration_ =
      config::RegistrationWithConfig<GraphColorAdaptor, FeatureScoreColor, Config>(
          "FeatureScoreColor");
};

void declare_config(FeatureScoreColor::Config& config);

class NearestFeatureColor : public GraphColorAdaptor {
 public:
  struct Config {
    config::VirtualConfig<EmbeddingDistance> metric{CosineDistance::Config()};
    config::VirtualConfig<EmbeddingGroup> features{RosEmbeddingGroup::Config()};
  } const config;

  explicit NearestFeatureColor(const Config& config);
  spark_dsg::Color getColor(const spark_dsg::DynamicSceneGraph& graph,
                            const spark_dsg::SceneGraphNode& node) const override;

 private:
  ros::NodeHandle nh_;
  std::unique_ptr<EmbeddingGroup> features_;
  std::unique_ptr<EmbeddingDistance> metric_;

  inline static const auto registration_ =
      config::RegistrationWithConfig<GraphColorAdaptor, NearestFeatureColor, Config>(
          "NearestFeatureColor");
};

void declare_config(NearestFeatureColor::Config& config);

}  // namespace hydra
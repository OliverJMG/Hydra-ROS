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
#include "hydra_ros/frontend/ros_frontend.h"

#include <config_utilities/config.h>
#include <config_utilities/printing.h>
#include <hydra/common/hydra_config.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

namespace hydra {

using message_filters::Subscriber;

void declare_config(RosFrontendConfig& conf) {
  using namespace config;
  name("RosFrontendConfig");
  base<FrontendConfig>(conf);
  field(conf.frontend_ns, "frontend_namespace");
  field(conf.enable_active_mesh_pub, "enable_active_mesh_pub");
  field(conf.enable_segmented_mesh_pub, "enable_segmented_mesh_pub");
  field(conf.odom_frame, "odom");
  field(conf.sensor_frame, "sensor_frame");
  field(conf.use_latest_tf, "use_latest_tf");
  field(conf.use_posegraph_pos, "use_posegraph_pos");
}

RosFrontend::RosFrontend(const RosFrontendConfig& config,
                         const SharedDsgInfo::Ptr& dsg,
                         const SharedModuleState::Ptr& state,
                         const LogSetup::Ptr& log_setup)
    : FrontendModule(config, dsg, state, log_setup),
      config_(config),
      nh_(config.frontend_ns) {
  pose_graph_sub_ = nh_.subscribe(
      "pose_graph_incremental", 100, &RosFrontend::poseGraphCallback, this);

  mesh_sub_.reset(new Subscriber<ActiveMesh>(nh_, "voxblox_mesh", 5));
  places_sub_.reset(new Subscriber<ActiveLayer>(nh_, "active_places", 5));
  sync_.reset(new Sync(Policy(10), *places_sub_, *mesh_sub_));
  sync_->registerCallback(boost::bind(&RosFrontend::inputCallback, this, _1, _2));

  tf_listener_.reset(new tf2_ros::TransformListener(buffer_));

  if (config_.enable_active_mesh_pub) {
    active_vertices_pub_ = nh_.advertise<MeshVertexCloud>("active_vertices", 1, true);
    segmenter_->addVisualizationCallback(
        [this](const auto& cloud, const auto& indices, const auto& labels) {
          this->publishActiveVertices(cloud, indices, labels);
        });
  }
  if (config_.enable_segmented_mesh_pub) {
    segmented_vertices_pub_.reset(new ObjectCloudPub("object_vertices", nh_));
    segmenter_->addVisualizationCallback(
        [this](const auto& cloud, const auto& indices, const auto& labels) {
          this->publishObjectClouds(cloud, indices, labels);
        });
  }
}

RosFrontend::~RosFrontend() { segmented_vertices_pub_.reset(); }

std::string RosFrontend::printInfo() const {
  std::stringstream ss;
  ss << config::toString(config_);
  return ss.str();
}

void RosFrontend::inputCallback(const ActiveLayer::ConstPtr& places,
                                const ActiveMesh::ConstPtr& mesh) {
  VLOG(5) << "Received input @ " << places->header.stamp.toNSec() << " [ns]";
  LOG(FATAL) << "ROS input for frontend not implemented currently";

  ReconstructionOutput::Ptr input(new ReconstructionOutput());

  if (config_.use_posegraph_pos && !pose_graph_queue_.empty()) {
    const auto latest_position = getLatestPosition();
    if (!latest_position) {
      ROS_ERROR_STREAM("Could not extract position from empty pose graph!");
      return;
    }

    input->current_position = *latest_position;
  } else {
    if (config_.use_posegraph_pos) {
      ROS_WARN_STREAM("Falling back to using tf for latest pos");
    }

    const auto latest_position = getLatestPositionTf(places->header.stamp);
    if (!latest_position) {
      ROS_ERROR_STREAM("Could not lookup latest position from tf!");
      return;
    }

    input->current_position = *latest_position;
  }

  // input->places = places;
  // input->mesh = mesh;
  input->timestamp_ns = places->header.stamp.toNSec();

  // send all cached messages to frontend
  input->pose_graphs = pose_graph_queue_;
  pose_graph_queue_.clear();

  queue_->push(input);
}

void RosFrontend::poseGraphCallback(const PoseGraph::ConstPtr& pose_graph) {
  pose_graph_queue_.push_back(pose_graph);
}

void RosFrontend::publishActiveVertices(const MeshVertexCloud& vertices,
                                        const MeshSegmenter::IndicesVector& indices,
                                        const LabelIndices&) const {
  MeshVertexCloud::Ptr active_cloud(new MeshVertexCloud());
  active_cloud->reserve(indices.size());
  for (const auto idx : indices) {
    active_cloud->push_back(vertices.at(idx));
  }

  active_cloud->header.frame_id = HydraConfig::instance().getFrames().odom;
  pcl_conversions::toPCL(ros::Time::now(), active_cloud->header.stamp);
  active_vertices_pub_.publish(active_cloud);
}

void RosFrontend::publishObjectClouds(const MeshVertexCloud& vertices,
                                      const MeshSegmenter::IndicesVector&,
                                      const LabelIndices& label_indices) const {
  for (const auto& label_index_pair : label_indices) {
    MeshVertexCloud label_cloud;
    label_cloud.reserve(label_index_pair.second->size());
    for (const auto idx : *label_index_pair.second) {
      label_cloud.push_back(vertices.at(idx));
    }

    label_cloud.header.frame_id = HydraConfig::instance().getFrames().odom;
    pcl_conversions::toPCL(ros::Time::now(), label_cloud.header.stamp);
    segmented_vertices_pub_->publish(label_index_pair.first, label_cloud);
  }
}

std::optional<Eigen::Vector3d> RosFrontend::getLatestPosition() const {
  const auto& msg = pose_graph_queue_.back();
  if (msg->nodes.empty()) {
    return std::nullopt;
  }

  const auto& pose = msg->nodes.back().pose;
  return Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
}

std::optional<Eigen::Vector3d> RosFrontend::getLatestPositionTf(
    const ros::Time& time_to_use) const {
  geometry_msgs::TransformStamped transform;
  try {
    transform =
        buffer_.lookupTransform(config_.odom_frame,
                                config_.sensor_frame,
                                config_.use_latest_tf ? ros::Time(0) : time_to_use);
  } catch (const tf2::TransformException& ex) {
    ROS_WARN_STREAM(ex.what());
    return std::nullopt;
  }

  return Eigen::Vector3d(transform.transform.translation.x,
                         transform.transform.translation.y,
                         transform.transform.translation.z);
}

}  // namespace hydra
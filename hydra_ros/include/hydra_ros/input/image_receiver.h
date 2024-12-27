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
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/msg/image.hpp>

#include "hydra_ros/input/ros_data_receiver.h"

namespace hydra {

using ImageSimpleFilter = message_filters::SimpleFilter<sensor_msgs::msg::Image>;

struct ImageSubImpl;

struct ColorSubscriber {
  ColorSubscriber();
  ColorSubscriber(rclcpp::Node::SharedPtr node, uint32_t queue_size = 1);
  virtual ~ColorSubscriber();

  ImageSimpleFilter& getFilter() const;
  void fillInput(const sensor_msgs::msg::Image& img, ImageInputPacket& packet) const;

 private:
  std::shared_ptr<ImageSubImpl> impl_;
};

struct DepthSubscriber {
  DepthSubscriber();
  DepthSubscriber(rclcpp::Node::SharedPtr node, uint32_t queue_size = 1);
  virtual ~DepthSubscriber();

  ImageSimpleFilter& getFilter() const;
  void fillInput(const sensor_msgs::msg::Image& img, ImageInputPacket& packet) const;

 private:
  std::shared_ptr<ImageSubImpl> impl_;
};

struct LabelSubscriber {
  using MsgType = sensor_msgs::msg::Image;
  LabelSubscriber();
  LabelSubscriber(rclcpp::Node::SharedPtr node, uint32_t queue_size = 1);
  virtual ~LabelSubscriber();

  ImageSimpleFilter& getFilter() const;
  void fillInput(const sensor_msgs::msg::Image& img, ImageInputPacket& packet) const;

 private:
  std::shared_ptr<ImageSubImpl> impl_;
};

template <typename SemanticT>
class ImageReceiverImpl : public RosDataReceiver {
 public:
  using SemanticMsgPtr = typename SemanticT::MsgType::ConstPtr;
  using Policy =
      message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image,
                                                      sensor_msgs::msg::Image,
                                                      typename SemanticT::MsgType>;
  using Synchronizer = message_filters::Synchronizer<Policy>;

  ImageReceiverImpl(const RosDataReceiver::Config& config,
                    const std::string& sensor_name);
  virtual ~ImageReceiverImpl() = default;

 protected:
  bool initImpl() override;

  void callback(const sensor_msgs::msg::Image::ConstSharedPtr& color,
                const sensor_msgs::msg::Image::ConstSharedPtr& depth,
                const SemanticMsgPtr& labels);

  ColorSubscriber color_sub_;
  DepthSubscriber depth_sub_;
  SemanticT semantic_sub_;
  std::unique_ptr<Synchronizer> sync_;
};

template <typename SemanticT>
ImageReceiverImpl<SemanticT>::ImageReceiverImpl(const Config& config,
                                                const std::string& sensor_name)
    : RosDataReceiver(config, sensor_name) {}

template <typename SemanticT>
bool ImageReceiverImpl<SemanticT>::initImpl() {
  color_sub_ = ColorSubscriber(node_);
  depth_sub_ = DepthSubscriber(node_);
  semantic_sub_ = SemanticT(node_);
  sync_.reset(new Synchronizer(Policy(config.queue_size),
                               color_sub_.getFilter(),
                               depth_sub_.getFilter(),
                               semantic_sub_.getFilter()));
  sync_->registerCallback(&ImageReceiverImpl<SemanticT>::callback, this);
  return true;
}

template <typename SemanticT>
void ImageReceiverImpl<SemanticT>::callback(const sensor_msgs::msg::Image::ConstSharedPtr& color,
                                            const sensor_msgs::msg::Image::ConstSharedPtr& depth,
                                            const SemanticMsgPtr& labels) {
  const auto timestamp_ns = rclcpp::Time(color->header.stamp).nanoseconds();
  if (!checkInputTimestamp(timestamp_ns)) {
    return;
  }

  auto packet = std::make_shared<ImageInputPacket>(timestamp_ns, sensor_name_);
  color_sub_.fillInput(*color, *packet);
  depth_sub_.fillInput(*depth, *packet);
  semantic_sub_.fillInput(*labels, *packet);
  queue.push(packet);
}

class ClosedSetImageReceiver : public ImageReceiverImpl<LabelSubscriber> {
 public:
  struct Config : RosDataReceiver::Config {};
  ClosedSetImageReceiver(const Config& config, const std::string& sensor_name);
  virtual ~ClosedSetImageReceiver() = default;

 private:
  inline static const auto registration_ =
    config::RegistrationWithConfig<DataReceiver,
                                   ClosedSetImageReceiver,
                                   ClosedSetImageReceiver::Config,
                                   std::string>("ClosedSetImageReceiver");
};

void declare_config(ClosedSetImageReceiver::Config& config);

}  // namespace hydra

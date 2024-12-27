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
#include "hydra_ros/utils/bag_reader.h"

#include <config_utilities/config.h>
#include <config_utilities/printing.h>
#include <config_utilities/types/path.h>
#include <config_utilities/validation.h>
#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <glog/logging.h>
#include <hydra/common/global_info.h>
#include <hydra/input/camera.h>
#include <hydra/input/input_packet.h>
#include <hydra/input/input_conversion.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <rosbag2_transport/reader_writer_factory.hpp>
// #include <rosbag/bag.h>
// #include <rosbag/view.h>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include "hydra_ros/utils/pose_cache.h"

namespace hydra {

using sensor_msgs::msg::Image;
using Policy = message_filters::sync_policies::ApproximateTime<Image, Image>;
using TimeSync = message_filters::Synchronizer<Policy>;

void declare_config(BagConfig& config) {
  using namespace config;
  name("BagConfig");
  field<Path>(config.bag_path, "bag_path");
  field(config.color_topic, "color_topic");
  field(config.depth_topic, "depth_topic");
  field(config.start, "start");
  field(config.duration, "duration");
  field(config.sensor, "sensor");
  field(config.sensor_frame, "sensor_frame");
  field(config.world_frame, "world_frame");
  check(config.color_topic, NE, "", "color_topic");
  check(config.depth_topic, NE, "", "depth_topic");
  check<Path::Exists>(config.bag_path, "bag_path");
}

Image::ConstSharedPtr getImageMessage(rosbag2_storage::SerializedBagMessageSharedPtr m) {
  rclcpp::SerializedMessage serialized_msg(*m->serialized_data);

  rclcpp::Serialization<sensor_msgs::msg::Image> raw_serialization;
  auto raw = std::make_shared<sensor_msgs::msg::Image>();
  raw_serialization.deserialize_message(&serialized_msg, raw.get());
  if (raw) {
    // no need to do anything special with normal image
    return raw;
  }

  rclcpp::Serialization<sensor_msgs::msg::CompressedImage> serialization;
  auto msg = std::make_shared<sensor_msgs::msg::CompressedImage>();
  serialization.deserialize_message(&serialized_msg, msg.get());
  if (!msg) {
    LOG(ERROR) << "Unable to parse image from '" << m->topic_name << "'";
    return nullptr;
  }

  const auto cv_ptr = cv_bridge::toCvCopy(msg);
  return cv_ptr->toImageMsg();
}

BagReader::BagReader(const Config& config)
    : config(config::checkValid(config)), sinks_(Sink::instantiate(config.sinks)) {}

void BagReader::read() {
  for (const auto& bag : config.bags) {
    readBag(bag);
  }
}

void BagReader::addSink(const Sink::Ptr& sink) {
  if (sink) {
    sinks_.push_back(sink);
  }
}

struct Trampoline {
  const BagConfig config;
  BagReader* reader;
  const PoseCache* cache;
  Sensor::ConstPtr sensor;

  void call(sensor_msgs::msg::Image::ConstSharedPtr msg1,
            sensor_msgs::msg::Image::ConstSharedPtr msg2) {
    reader->handleImages(config, sensor, *cache, msg1, msg2);
  }
};

void BagReader::readBag(const BagConfig& bag_config) {
  LOG(INFO) << "Reading bag from config: " << std::endl << config::toString(bag_config);
  std::vector<std::string> topics{bag_config.color_topic, bag_config.depth_topic};

  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = bag_config.bag_path;

  const Sensor::ConstPtr sensor = bag_config.sensor.create();
  if (!sensor) {
    LOG(ERROR) << "Could not load sensor for bag " << bag_config.bag_path;
    return;
  }

  PoseCache cache(storage_options);
  Trampoline trampoline{bag_config, this, &cache, sensor};

  TimeSync sync(Policy(10));
  sync.registerCallback(std::bind(&Trampoline::call, &trampoline,
          std::placeholders::_1, std::placeholders::_2));

  rclcpp::Time start;
  bool have_start = false;
  
  auto reader = rosbag2_transport::ReaderWriterFactory::make_reader(storage_options);
  reader->open(storage_options);

  rosbag2_storage::StorageFilter filter;
  filter.topics = topics;
  reader->set_filter(filter);

  while (reader->has_next()) {
    rosbag2_storage::SerializedBagMessageSharedPtr msg = reader->read_next();
    rclcpp::Time msg_time{msg->send_timestamp};

    if (!have_start) {
      start = msg_time;
      if (bag_config.start >= 0.0) {
        start += rclcpp::Duration::from_seconds(bag_config.start);
      }
      have_start = true;
    }

    const auto diff_s = (msg_time - start).seconds();
    if (diff_s < 0.0) {
      VLOG(2) << "Skipping message " << std::abs(diff_s) << " [s] before start";
      continue;
    }

    if (bag_config.duration >= 0.0 && diff_s > bag_config.duration) {
      LOG(INFO) << "Reached end of duration: " << diff_s << " [s]";
      return;
    }

    const auto topic = msg->topic_name;
    
    auto img_msg = getImageMessage(msg);
    if (!img_msg) {
      continue;
    }

    if (topic == bag_config.color_topic) {
      VLOG(10) << "new " << bag_config.color_topic << " @ "
               << msg_time.nanoseconds();
      sync.add<0>(message_filters::MessageEvent<Image>(img_msg, msg_time));
    } else {
      VLOG(10) << "new " << bag_config.depth_topic << " @ "
               << msg_time.nanoseconds();
      sync.add<1>(message_filters::MessageEvent<Image>(img_msg, msg_time));
    }
  }

  reader->close();
}

void BagReader::handleImages(const BagConfig& bag_config,
                             const Sensor::ConstPtr& sensor,
                             const PoseCache& cache,
                             sensor_msgs::msg::Image::ConstSharedPtr color_msg,
                             sensor_msgs::msg::Image::ConstSharedPtr depth_msg) {
  if (!sensor) {
    LOG(ERROR) << "sensor required!";
    return;
  }

  const auto timestamp_ns = rclcpp::Time(color_msg->header.stamp).nanoseconds();
  VLOG(5) << "processing images @ " << timestamp_ns << " [ns]";

  const auto sensor_frame = !bag_config.sensor_frame.empty()
                                ? bag_config.sensor_frame
                                : color_msg->header.frame_id;
  const auto world_frame = !bag_config.world_frame.empty()
                               ? bag_config.world_frame
                               : GlobalInfo::instance().getFrames().odom;
  const auto pose = cache.lookupPose(timestamp_ns, world_frame, sensor_frame);
  if (!pose) {
    LOG(ERROR) << "Could not find pose for data @ " << timestamp_ns << " [ns]";
    return;
  }

  InputData data(sensor);
  data.timestamp_ns = timestamp_ns;
  data.world_T_body = pose.to_T_from();
  data.color_image = cv_bridge::toCvCopy(color_msg)->image.clone();
  cv::cvtColor(data.color_image, data.color_image, cv::COLOR_BGR2RGB);
  data.depth_image = cv_bridge::toCvCopy(depth_msg)->image.clone();

  const auto valid = conversions::normalizeData(data, false);
  if (!valid) {
    LOG(ERROR) << "Failed to normalize frame data @ " << data.timestamp_ns << " [ns]";
    return;
  }

  if (!sensor->finalizeRepresentations(data)) {
    LOG(ERROR) << "Failed to finalized data @ " << data.timestamp_ns << " [ns]";
    return;
  }

  Sink::callAll(sinks_, data);
}

void declare_config(BagReader::Config& config) {
  using namespace config;
  name("BagReader::Config");
  field(config.bags, "bags");
  field(config.sinks, "sinks");
}

}  // namespace hydra

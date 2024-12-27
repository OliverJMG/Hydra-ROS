#include "hydra_ros/input/pointcloud_receiver.h"

#include <config_utilities/config.h>
#include <glog/logging.h>
#include <hydra/common/global_info.h>

#include "hydra_ros/input/pointcloud_adaptor.h"

namespace hydra {

void declare_config(PointcloudReceiver::Config& config) {
  using namespace config;
  name("PointcloudReceiver::Config");
  base<RosDataReceiver::Config>(config);
}

PointcloudReceiver::PointcloudReceiver(const Config& config,
                                       const std::string& sensor_name)
    : RosDataReceiver(config, sensor_name), config(config) {}

bool PointcloudReceiver::initImpl() {
  sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      "pointcloud", config.queue_size, 
      std::bind(&PointcloudReceiver::callback, this, std::placeholders::_1));
  return true;
}

void PointcloudReceiver::callback(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
  const auto timestamp_ns = rclcpp::Time(msg->header.stamp).nanoseconds();
  VLOG(5) << "[Hydra Reconstruction] Got raw pointcloud input @ " << timestamp_ns
          << " [ns]";

  if (!checkInputTimestamp(timestamp_ns)) {
    return;
  }

  auto packet = std::make_shared<CloudInputPacket>(timestamp_ns, sensor_name_);
  fillPointcloudPacket(*msg, *packet, false);
  // TODO(nathan) this is brittle, but at least handles kitti
  packet->in_world_frame =
      msg->header.frame_id == GlobalInfo::instance().getFrames().odom;
  queue.push(packet);
}

}  // namespace hydra

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
#include <rclcpp/rclcpp.hpp>

#include <map>
#include <string>

namespace hydra {

template <typename Derived>
struct publisher_type_trait;

// NOTE(nathan) CRTP or something similar seems necessary here becuase the various
// publisher types need something common (the Derived class) to hold construction
// information
template <typename Derived>
class LazyPublisherGroup {
 public:
  virtual ~LazyPublisherGroup() = default;

  template <typename Callback>
  void publish(const std::string& topic, const Callback callback) const {
    const auto derived = static_cast<const Derived*>(this);
    auto iter = pubs_.find(topic);
    if (iter == pubs_.end()) {
      iter = pubs_.emplace(topic, derived->makePublisher(topic)).first;
    }

    if (!derived->shouldPublish(iter->second)) {
      return;
    }

    derived->publishMsg(iter->second, callback());
  }

 private:
  friend Derived;
  LazyPublisherGroup() = default;

  using PublisherT = typename publisher_type_trait<Derived>::value;
  mutable std::map<std::string, PublisherT> pubs_;
};

template <typename T>
struct RosPublisherGroup;

template <typename T>
struct publisher_type_trait<RosPublisherGroup<T>> {
  using value = typename rclcpp::Publisher<T>::SharedPtr;
};

template <typename T>
struct RosPublisherGroup : LazyPublisherGroup<RosPublisherGroup<T>> {
 public:
  using Base = LazyPublisherGroup<RosPublisherGroup<T>>;

  explicit RosPublisherGroup(const std::string& name, const std::string& ns) {
    node_ = std::make_shared<rclcpp::Node>(name, ns);
    param_cb_ = node_->add_post_set_parameters_callback(std::bind(
            &RosPublisherGroup<T>::on_param_change, this, std::placeholders::_1));
    node_->declare_parameter<int>("queue_size", 1);
    node_->declare_parameter<bool>("latch", false);
  }

  typename rclcpp::Publisher<T>::SharedPtr makePublisher(const std::string& topic) const {
    rclcpp::QoS qos = rclcpp::QoS(queue_size);
    if(latch) qos = qos.transient_local();
    return node_->create_publisher<T>(topic, qos);
  }

  bool shouldPublish(typename rclcpp::Publisher<T>::SharedPtr pub) const {
    return pub->get_subscription_count() > 0;
  }

  void publishMsg(typename rclcpp::Publisher<T>::SharedPtr& pub, T& msg) const { 
    pub->publish(msg); 
  }

  void publishMsg(typename rclcpp::Publisher<T>::SharedPtr& pub, typename T::UniquePtr msg) const {
    pub->publish(std::move(msg));
  }

  size_t queue_size;
  bool latch;

 private:

  void on_param_change(const std::vector<rclcpp::Parameter> & parameters) { 
    for(const auto & param:parameters) { 
      // Update internal class parameter values
      if (param.get_name() == "queue_size") {
        queue_size = param.as_int();
      } else if (param.get_name() == "latch") {
        latch = param.as_bool();
      }
    } 
  }

  mutable rclcpp::Node::SharedPtr node_;
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr param_cb_;
};

}  // namespace hydra
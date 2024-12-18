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
#include <config_utilities/config_utilities.h>
#include <config_utilities/external_registry.h>
#include <config_utilities/logging/log_to_stdout.h>
#include <config_utilities/settings.h>
#include <config_utilities/parsing/ros2.h>
#include <glog/logging.h>

#include <filesystem>

#include "hydra_visualizer/dsg_visualizer.h"

namespace hydra::visualizer {
struct ExternalPluginConfig {
  bool allow_plugins = true;
  bool verbose_plugins = false;
  bool trace_plugin_allocations = false;
  std::vector<std::string> paths;
};

void declare_config(ExternalPluginConfig& config) {
  using namespace config;
  name("ExternalPluginConfig");
  field(config.allow_plugins, "allow_plugins");
  field(config.verbose_plugins, "verbose_plugins");
  field(config.trace_plugin_allocations, "trace_plugin_allocations");
  field(config.paths, "paths");
}

}  // namespace hydra::visualizer

int main(int argc, char** argv) {

  // Strip ROS specific arguments that gflags can't parse
  std::vector<std::string> non_ros_args = rclcpp::remove_ros_arguments(argc, argv);

  // Create argc and argv equivalents
  int non_ros_argc = non_ros_args.size();
  char ** non_ros_argv = new char*[non_ros_argc];
  for (size_t i = 0; i < non_ros_args.size(); ++i) {
    non_ros_argv[i] = new char[non_ros_args.at(i).size() + 1]; // +1 for null-terminator
    strcpy(non_ros_argv[i], non_ros_args.at(i).c_str());
  }

  google::InitGoogleLogging(non_ros_argv[0]);
  google::ParseCommandLineFlags(&non_ros_argc, &non_ros_argv, false);
  google::InstallFailureSignalHandler();

  for (size_t i = 0; i < non_ros_args.size(); ++i) {
    delete [] non_ros_argv[i];
  }
  delete [] non_ros_argv;

  rclcpp::init(argc, argv);

  FLAGS_minloglevel = 0;
  FLAGS_logtostderr = 1;
  FLAGS_colorlogtostderr = 1;

  const auto plugin_config =
      config::fromYamlFile<hydra::visualizer::ExternalPluginConfig>(
              "/workspaces/ros2_ws/src/hydra_ros/hydra_visualizer/config/external_plugins.yaml", "external_plugins");
  RCLCPP_INFO_STREAM(rclcpp::get_logger("hydra_visualizer_node"), "Plugins:\n" << config::toString(plugin_config));

  auto& settings = config::Settings();
  settings.allow_external_libraries = plugin_config.allow_plugins;
  settings.verbose_external_load = plugin_config.verbose_plugins;
  settings.print_external_allocations = plugin_config.trace_plugin_allocations;
  const auto plugins = config::loadExternalFactories(plugin_config.paths);

  {  // start visualizer scope
    // const auto config = config::fromRos<hydra::DsgVisualizer::Config>(nh);
    // RCLCPP_INFO_STREAM(rclcpp::get_logger("hydra_visualizer_node"), "Config:\n" << config::toString(config));
    auto node = std::make_shared<hydra::DsgVisualizer>();
    node->configure();
    node->start();
    rclcpp::spin(node);
  }  // end visualizer scope

  return 0;
}

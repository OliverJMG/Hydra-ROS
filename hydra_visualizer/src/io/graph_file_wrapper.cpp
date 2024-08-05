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
#include "hydra_visualizer/io/graph_file_wrapper.h"

#include <config_utilities/config.h>
#include <config_utilities/types/path.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>

namespace hydra {

using spark_dsg::DynamicSceneGraph;

void declare_config(GraphFileWrapper::Config& config) {
  using namespace config;
  name("GraphFileWrapper::Config");
  field<Path>(config.filepath, "filepath");
  field(config.wrapper_ns, "wrapper_ns");

  check<Path::Exists>(config.filepath, "filepath");
}

GraphFileWrapper::GraphFileWrapper(const Config& config)
    : config(config::checkValid(config)),
      nh_(config.wrapper_ns),
      has_change_(true),
      filepath_(config.filepath),
      graph_(DynamicSceneGraph::load(filepath_)),
      service_(nh_.advertiseService("reload", &GraphFileWrapper::reload, this)),
      sub_(nh_.subscribe("load", 1, &GraphFileWrapper::load, this)) {}

bool GraphFileWrapper::hasChange() const { return has_change_; }

void GraphFileWrapper::clearChangeFlag() { has_change_ = false; }

DynamicSceneGraph::Ptr GraphFileWrapper::get() const { return graph_; }

bool GraphFileWrapper::reload(std_srvs::Empty::Request&, std_srvs::Empty::Response&) {
  graph_ = DynamicSceneGraph::load(filepath_);
  has_change_ = true;
  return true;
}

void GraphFileWrapper::load(const std_msgs::String& msg) {
  std::filesystem::path req_path(msg.data);
  if (!std::filesystem::exists(req_path)) {
    LOG(ERROR) << "Graph does not exist at '" << req_path.string() << "'";
    return;
  }

  filepath_ = req_path;
  graph_ = DynamicSceneGraph::load(filepath_);
  has_change_ = true;
}

}  // namespace hydra

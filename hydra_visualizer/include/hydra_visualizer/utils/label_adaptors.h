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
#include <spark_dsg/dynamic_scene_graph.h>
#include <config_utilities/factory.h>

namespace hydra::visualizer {

struct GraphLabelAdaptor {
  using Ptr = std::shared_ptr<GraphLabelAdaptor>;
  virtual ~GraphLabelAdaptor() = default;
  virtual std::string getLabel(const spark_dsg::SceneGraphNode& node) const = 0;
};

struct IdLabelAdaptor : GraphLabelAdaptor {
  struct Config {};
  explicit IdLabelAdaptor(const Config&) {}
  virtual ~IdLabelAdaptor() = default;
  std::string getLabel(const spark_dsg::SceneGraphNode& node) const override;

  inline static const auto id_reg =
    config::RegistrationWithConfig<GraphLabelAdaptor,
                                   IdLabelAdaptor,
                                   IdLabelAdaptor::Config>("IdLabelAdaptor");
};

void declare_config(IdLabelAdaptor::Config& config);

struct NameLabelAdaptor : GraphLabelAdaptor {
  struct Config {};
  explicit NameLabelAdaptor(const Config&) {}
  virtual ~NameLabelAdaptor() = default;
  std::string getLabel(const spark_dsg::SceneGraphNode& node) const override;

  inline static const auto name_reg =
    config::RegistrationWithConfig<GraphLabelAdaptor, NameLabelAdaptor, NameLabelAdaptor::Config>(
        "NameLabelAdaptor");
};

void declare_config(NameLabelAdaptor::Config& config);

}  // namespace hydra::visualizer
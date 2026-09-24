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

#include <config_utilities/virtual_config.h>
#include <pybind11/pybind11.h>

#include <memory>
#include <string>

#include "hydra/active_window/active_window_module.h"
#include "hydra/active_window/reconstruction_module.h"
#include "hydra/backend/backend_module.h"
#include "hydra/common/global_info.h"
#include "hydra/common/hydra_pipeline.h"
#include "hydra/frontend/graph_builder.h"
#include "hydra/input/input_packet.h"
#include "hydra/input/sensor.h"
#include "hydra/loop_closure/loop_closure_config.h"
#include "hydra/loop_closure/loop_closure_module.h"

namespace hydra::python {

// Base, flow-agnostic Hydra pipeline exposed to Python (registered as
// "HydraPipeline"). KAIROS subclasses this (see kairos::KairosPipeline) to add
// the flow/temporal module without Hydra depending on KAIROS. The flow-specific
// wiring is injected through the protected virtual hooks below.
class PythonPipeline : public HydraPipeline {
 public:
  struct Config : PipelineConfig {
    config::VirtualConfig<ActiveWindowModule> active_window{
        ReconstructionModule::Config()};
    config::VirtualConfig<GraphBuilder> frontend{GraphBuilder::Config()};
    config::VirtualConfig<BackendModule> backend{BackendModule::Config()};
    LoopClosureConfig loop_closure;
  } const config;

  PythonPipeline(const Config& config,
                 const Sensor::Ptr& sensor,
                 int robot_id = 0,
                 int config_verbosity = 0,
                 bool freeze_global_info = true,
                 bool step_mode_only = true,
                 std::string zmq_url = "");

  virtual ~PythonPipeline();

  void start() override;
  void stop() override;
  void save() override;
  void reset();
  bool step(const InputPacket::Ptr& input);

  DynamicSceneGraph::Ptr getSceneGraph() const;

  const std::string sensor_name;
  const std::string zmq_url;

 protected:
  // Hooks for subclasses (e.g. KAIROS) to attach extra modules/workers without
  // Hydra knowing about them. Called from the corresponding base methods.
  virtual void initExtraModules() {}
  virtual void startExtraWorkers() {}
  virtual void stopExtraWorkers() {}
  virtual void resetExtraModules() {}

  void initModules();
  void startWorkers();
  void stopWorkers();

  bool step_mode_only_;
  bool workers_started_ = false;
  std::shared_ptr<ActiveWindowModule> active_window_;
  std::shared_ptr<GraphBuilder> frontend_;
  std::shared_ptr<BackendModule> backend_;
  std::shared_ptr<LoopClosureModule> loop_closure_;

  DynamicSceneGraph::Ptr graph_;
};

void declare_config(PythonPipeline::Config& config);

namespace python_pipeline {
void addBindings(pybind11::module_& m);
}

}  // namespace hydra::python

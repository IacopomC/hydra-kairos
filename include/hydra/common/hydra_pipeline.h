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
#include "hydra/common/global_info.h"
#include "hydra/common/module.h"
#include "hydra/common/shared_module_state.h"

/**
 * @file hydra_pipeline.h
 * @brief Incremental, multi-module pipeline. Owns the shared scene-graph info
 *        and module state, holds the registered modules by name, and drives
 *        their lifecycle (init, start, stop, save) as a group.
 */

namespace hydra {

/// @brief Coordinates the lifecycle of the modules that make up the pipeline.
/// Initializes global info on construction and owns the shared scene-graph
/// containers and module state passed to each module.
class HydraPipeline {
 public:
  /// @brief Construct the pipeline and initialize global info.
  /// @param config Pipeline-wide configuration.
  /// @param robot_id Identifier used to derive the robot prefix.
  /// @param config_verbosity Verbosity for config printing.
  /// @param freeze_global_info If true, freeze global info after initialization.
  HydraPipeline(const PipelineConfig& config,
                int robot_id = 0,
                int config_verbosity = 1,
                bool freeze_global_info = true);

  virtual ~HydraPipeline();

  /// @brief Construct and wire up the modules.
  virtual void init();

  /// @brief Start all modules.
  virtual void start();

  /// @brief Stop all modules.
  virtual void stop();

  /// @brief Save module outputs to the configured log location.
  virtual void save();

  /// @brief Look up a registered module by name and cast it to a derived type.
  /// @tparam Derived Expected concrete module type.
  /// @param name Name the module was registered under.
  /// @return Pointer to the module, or nullptr if absent or of the wrong type.
  template <typename Derived = Module>
  Derived* getModule(const std::string& name) {
    auto iter = modules_.find(name);
    if (iter == modules_.end()) {
      return nullptr;
    }

    return dynamic_cast<Derived*>(iter->second.get());
  }

 protected:
  /// @brief Log a summary of the registered modules.
  void showModules() const;

  /// @brief Build a descriptive string for a single named module.
  /// @param name Name the module is registered under.
  /// @param module Module to describe.
  /// @return Human-readable description of the module.
  std::string getModuleInfo(const std::string& name, const Module* module) const;

 protected:
  int config_verbosity_;              ///< Verbosity used when printing configs.
  SharedDsgInfo::Ptr frontend_dsg_;   ///< Scene graph shared with the frontend.
  SharedDsgInfo::Ptr backend_dsg_;    ///< Scene graph shared with the backend.
  SharedModuleState::Ptr shared_state_;  ///< State shared across modules.

  Module::Ptr input_module_;          ///< Module producing pipeline input.
  std::map<std::string, Module::Ptr> modules_;  ///< Registered modules keyed by name.
};

}  // namespace hydra

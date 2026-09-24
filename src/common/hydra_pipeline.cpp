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
#include "hydra/common/hydra_pipeline.h"

#include <config_utilities/settings.h>
#include <config_utilities/config.h>
#include <config_utilities/parsing/ros.h>
#include <config_utilities/printing.h>
#include <config_utilities/validation.h>

namespace hydra {

HydraPipeline::HydraPipeline(const PipelineConfig& pipeline_config,
                             int robot_id,
                             int config_verbosity,
                             bool freeze_global_info)
    : config_verbosity_(config_verbosity) {
  // seed the process-wide config first; every shared graph below is built from it
  const auto& config = GlobalInfo::init(pipeline_config, robot_id, freeze_global_info);
  // frontend and backend each own a separate graph so they can run concurrently;
  // the shared state holds the additional graphs the loop-closure and backend
  // threads operate on
  frontend_dsg_ = config.createSharedDsg();
  backend_dsg_ = config.createSharedDsg();
  shared_state_.reset(new SharedModuleState());
  shared_state_->lcd_graph = config.createSharedDsg();
  shared_state_->backend_graph = config.createSharedDsg();
}

void HydraPipeline::init() {}

HydraPipeline::~HydraPipeline() {}

// format a single log banner line padded to print_width with the fill char
std::string makeBanner(const std::string& message,
                       size_t print_width,
                       char fill,
                       bool with_header = true,
                       bool with_footer = false) {
  std::stringstream ss;
  if (with_header) {
    ss << std::string(print_width, fill) << std::endl;
  }
  // account for the leading fill, two surrounding spaces, and trailing fill when
  // computing right padding; clamp to zero if the message already overflows the width
  const auto msg_size = message.size() + 3;
  const auto spacing = msg_size >= print_width ? 0 : print_width - msg_size;
  ss << fill << " " << message << (spacing ? std::string(spacing, ' ') + fill : "")
     << std::endl;
  if (with_footer) {
    ss << std::string(print_width, fill) << std::endl;
  }
  return ss.str();
}

std::string HydraPipeline::getModuleInfo(const std::string& name,
                                         const Module* module) const {
  const auto print_width = config::Settings().print_width;
  std::stringstream ss;
  ss << makeBanner(name, print_width, '*', true, true);
  if (!module) {
    ss << "UNITIALIZED MODULE!" << std::endl;
  } else {
    const auto info = module->printInfo();
    if (!info.empty()) {
      ss << info << std::endl;
    }
  }
  ss << std::string(print_width, '*') << std::endl;
  return ss.str();
}

void HydraPipeline::showModules() const {
  const auto print_width = config::Settings().print_width;
  std::stringstream ss;
  ss << std::endl << makeBanner("Modules", print_width, '=', true, true);
  for (auto&& [name, module] : modules_) {
    ss << std::endl << getModuleInfo(name, module.get());
  }
  VLOG(config_verbosity_) << ss.str();
}

void HydraPipeline::start() {
  VLOG(config_verbosity_) << std::endl << getModuleInfo("input", input_module_.get());
  showModules();

  // start the input source first so downstream modules have data to consume
  if (input_module_) {
    input_module_->start();
  }

  for (auto&& [name, module] : modules_) {
    if (!module) {
      LOG(FATAL) << "Found unitialized module: " << name;
      continue;
    }

    module->start();
  }

  // register the input module alongside the rest so stop()/save() iterate over it too
  if (input_module_) {
    modules_["input"] = input_module_;
  }
}

void HydraPipeline::stop() {
  for (auto&& [name, module] : modules_) {
    if (!module) {
      LOG(FATAL) << "Found unitialized module: " << name;
      continue;
    }

    module->stop();
  }
}

void HydraPipeline::save() {
  // nothing to write without a valid log directory, so skip the whole pass
  const auto& logs = GlobalInfo::instance().getLogs();
  if (!logs || !logs->valid()) {
    return;
  }

  for (auto&& [name, module] : modules_) {
    if (!module) {
      LOG(FATAL) << "Found unitialized module: " << name;
      continue;
    }

    module->save(*logs);
  }
}

}  // namespace hydra

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
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
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
#include <config_utilities/config.h>
#include <config_utilities/factory.h>
#include <gtest/gtest.h>
#include <hydra/common/hydra_pipeline.h>

namespace hydra {

namespace {

struct CountingModule : public Module {
  struct Config {
    int value = 0;
  } const config;

  explicit CountingModule(const Config& config) : config(config) {}

  void start() override { ++num_started; }
  void stop() override { ++num_stopped; }

  size_t num_started = 0;
  size_t num_stopped = 0;

  inline static const auto registration_ =
      config::RegistrationWithConfig<Module, CountingModule, Config>("CountingModule");
};

void declare_config(CountingModule::Config& config) {
  using namespace config;
  name("CountingModule::Config");
  field(config.value, "value");
}

struct TestablePipeline : public HydraPipeline {
  TestablePipeline() : HydraPipeline(PipelineConfig()) {}
  ~TestablePipeline() override { GlobalInfo::instance().reset(); }

  using HydraPipeline::addModules;
};

}  // namespace

TEST(HydraPipeline, AddsConfiguredModules) {
  TestablePipeline pipeline;

  HydraPipeline::ModuleConfigs configs;
  configs.emplace_back("extra", config::VirtualConfig<Module>(CountingModule::Config{3}));
  pipeline.addModules(configs);

  auto mod = pipeline.getModule<CountingModule>("extra");
  ASSERT_TRUE(mod);
  EXPECT_EQ(mod->config.value, 3);

  // configured modules are started and stopped with the rest of the pipeline
  pipeline.start();
  pipeline.stop();
  EXPECT_EQ(mod->num_started, 1u);
  EXPECT_EQ(mod->num_stopped, 1u);

  // names are unique
  EXPECT_THROW(pipeline.addModules(configs), std::runtime_error);
}

}  // namespace hydra

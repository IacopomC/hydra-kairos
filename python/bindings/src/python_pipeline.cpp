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
#include "hydra/bindings/python_pipeline.h"

#include <config_utilities/config.h>
#include <config_utilities/parsing/yaml.h>
#include <config_utilities/printing.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>
#include <hydra/active_window/reconstruction_module.h>
#include <hydra/backend/backend_module.h>
#include <hydra/backend/zmq_interfaces.h>
#include <hydra/common/global_info.h>
#include <hydra/common/hydra_pipeline.h>
#include <hydra/common/pipeline_queues.h>
#include <hydra/frontend/graph_builder.h>
#include <hydra/input/camera.h>
#include <hydra/loop_closure/bow_agent_solver.h>
#include <hydra/loop_closure/loop_closure_config.h>
#include <hydra/loop_closure/loop_closure_module.h>
#include <pose_graph_tools/bow_query.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/stl/filesystem.h>
#include <pybind11/stl_bind.h>

#include "hydra/bindings/glog_utilities.h"
#include "hydra/bindings/python_image.h"
#include "hydra/bindings/python_sensor_input.h"
#include "hydra/bindings/python_sensors.h"

namespace hydra::python {

void declare_config(PythonPipeline::Config& config) {
  using namespace config;
  name("PythonPipeline::Config");
  base<PipelineConfig>(config);
  field(config.active_window, "active_window");
  field(config.frontend, "frontend");
  field(config.backend, "backend");
  // Inline so LoopClosureConfig's top-level fields and its nested `lcd:`
  // detector subblock parse from the same root as the rest of the pipeline
  // config (matches how `logs`/`frames` are merged in PipelineConfig).
  field(config.loop_closure, "loop_closure", false);
}

PythonPipeline::PythonPipeline(const Config& _config,
                               const Sensor::Ptr& sensor,
                               int robot_id,
                               int config_verbosity,
                               bool freeze_global_info,
                               bool step_mode_only,
                               std::string zmq_url)
    : HydraPipeline(_config, robot_id, config_verbosity, freeze_global_info),
      config(_config),
      sensor_name(sensor ? sensor->name : ""),
      zmq_url(zmq_url),
      step_mode_only_(step_mode_only) {
  if (!sensor) {
    throw std::runtime_error("Invalid sensor!");
  }

  GlogSingleton::instance().setLogLevel(0, 0, false);
  config::Settings().print_width = 100;
  config::Settings().print_indent = 45;

  GlobalInfo::instance().setSensor(sensor);
  VLOG(config_verbosity) << "\n"
                         << config::toString(GlobalInfo::instance().getConfig());

  initModules();

  // In step mode, callers that don't explicitly call start() still need the
  // worker-owning modules (loop_closure, and any subclass extras) running.
  if (step_mode_only_) {
    startWorkers();
  }
}

PythonPipeline::~PythonPipeline() { stop(); }

void PythonPipeline::start() {
  if (step_mode_only_) {
    LOG(INFO) << "[Hydra Python] Running in step mode!";
    startWorkers();
  } else {
    LOG(INFO) << "[Hydra Python] Running in parallel";
    HydraPipeline::start();
  }
}

void PythonPipeline::initModules() {
  const auto& logs = GlobalInfo::instance().getLogs();
  frontend_ = config.frontend.create(frontend_dsg_, shared_state_, logs);
  if (!frontend_) {
    throw std::runtime_error("Invalid frontend config!");
  }

  modules_["frontend"] = frontend_;
  backend_ = config.backend.create(backend_dsg_, shared_state_, logs);
  modules_["backend"] = backend_;
  active_window_ = config.active_window.create(frontend_->queue());
  modules_["reconstruction"] = active_window_;

  if (!zmq_url.empty()) {
    ZmqSink::Config zmq_config{zmq_url, true};
    backend_->addSink(std::make_shared<ZmqSink>(zmq_config));
  }

  if (GlobalInfo::instance().getConfig().enable_lcd) {
    auto lcd_config = config.loop_closure;
    lcd_config.detector.num_semantic_classes =
        GlobalInfo::instance().getTotalLabels();
    VLOG(1) << "Number of classes for LCD: "
            << lcd_config.detector.num_semantic_classes;
    config::checkValid(lcd_config);

    loop_closure_ = std::make_shared<LoopClosureModule>(lcd_config, shared_state_);
    modules_["lcd"] = loop_closure_;

    if (lcd_config.detector.enable_agent_registration) {
      loop_closure_->getDetector().setRegistrationSolver(
          0, std::make_unique<lcd::BowAgentSolver>(lcd_config.bow_agent));
    }
  }

  // Subclass hook (KAIROS): create + wire extra modules (e.g. the flow
  // temporal module) against active_window_/backend_ before they spin up.
  initExtraModules();

  showModules();
  VLOG(config_verbosity_) << GlobalInfo::instance();
}

void PythonPipeline::startWorkers() {
  if (workers_started_) {
    return;
  }
  startExtraWorkers();
  if (loop_closure_) {
    loop_closure_->start();
  }
  workers_started_ = true;
}

void PythonPipeline::stopWorkers() {
  if (!workers_started_) {
    return;
  }
  stopExtraWorkers();
  if (loop_closure_) {
    loop_closure_->stop();
  }
  workers_started_ = false;
}

void PythonPipeline::stop() {
  if (step_mode_only_) {
    stopWorkers();
    return;
  }

  HydraPipeline::stop();
}

void PythonPipeline::reset() {
  stop();

  // reset specific module instances
  active_window_.reset();
  frontend_.reset();
  backend_.reset();
  loop_closure_.reset();
  resetExtraModules();

  // reset any other modules (will actually deconstruct specific module
  // instances given shared_ptr usage)
  modules_.clear();

  // reset state
  const auto& config = GlobalInfo::instance();
  frontend_dsg_ = config.createSharedDsg();
  backend_dsg_ = config.createSharedDsg();

  // setup dependent graphs
  shared_state_.reset(new SharedModuleState());
  shared_state_->lcd_graph = config.createSharedDsg();
  shared_state_->backend_graph = config.createSharedDsg();

  PipelineQueues::instance().clear();

  initModules();

  if (!step_mode_only_) {
    start();
  } else {
    startWorkers();
  }
}

void PythonPipeline::save() {
  HydraPipeline::save();
  GlobalInfo::exit();
}

bool PythonPipeline::step(const InputPacket::Ptr& input) {
  if (!active_window_->step(input)) {
    return false;
  }

  if (!frontend_->spinOnce()) {
    LOG(ERROR) << "[Hydra] Frontend failed to return output";
    return false;
  }

  backend_->step(false);
  return true;
}

DynamicSceneGraph::Ptr PythonPipeline::getSceneGraph() const {
  return backend_dsg_->graph->clone();
}

namespace python_pipeline {

using namespace pybind11::literals;
namespace py = pybind11;

void addBindings(pybind11::module_& m) {
  py::class_<PythonPipeline>(m, "HydraPipeline")
      .def_static(
          "from_config",
          [](const std::string& contents,
             const Sensor::Ptr& sensor,
             int robot_id,
             int config_verbosity,
             bool freeze_global_info,
             bool step_mode_only,
             const std::string zmq_url) {
            const auto node = YAML::Load(contents);
            return std::make_unique<PythonPipeline>(
                config::fromYaml<PythonPipeline::Config>(node),
                sensor,
                robot_id,
                config_verbosity,
                freeze_global_info,
                step_mode_only,
                zmq_url);
          },
          "config"_a,
          "camera"_a,
          "robot_id"_a = 0,
          "config_verbosity"_a = 0,
          "freeze_global_info"_a = true,
          "use_step_mode"_a = true,
          "zmq_url"_a = "")
      .def_static(
          "from_file",
          [](const std::filesystem::path& filepath,
             const Sensor::Ptr& sensor,
             int robot_id,
             int config_verbosity,
             bool freeze_global_info,
             bool step_mode_only,
             const std::string& zmq_url) {
            const auto node = YAML::LoadFile(filepath);
            return std::make_unique<PythonPipeline>(
                config::fromYaml<PythonPipeline::Config>(node),
                sensor,
                robot_id,
                config_verbosity,
                freeze_global_info,
                step_mode_only,
                zmq_url);
          },
          "config"_a,
          "camera"_a,
          "robot_id"_a = 0,
          "config_verbosity"_a = 0,
          "freeze_global_info"_a = true,
          "use_step_mode"_a = true,
          "zmq_url"_a = "")
      .def_static("default_config",
                  []() {
                    std::stringstream ss;
                    ss << config::toYaml(PythonPipeline::Config());
                    return ss.str();
                  })
      .def("save", &PythonPipeline::save)
      .def("reset", &PythonPipeline::reset)
      .def(
          "step",
          [](PythonPipeline& pipeline,
             size_t timestamp_ns,
             const Eigen::Vector3d& world_t_body,
             const Eigen::Vector4d& world_R_body,
             const py::buffer& depth,
             const py::buffer& labels,
             const py::buffer& rgb,
             const FeatureVector& feature) {
            auto input = std::make_shared<InputPacket>();
            input->timestamp_ns = timestamp_ns;
            input->world_t_body = world_t_body;
            input->world_R_body = Eigen::Quaterniond(
                world_R_body[0], world_R_body[1], world_R_body[2], world_R_body[3]);
            input->sensor_input = std::make_unique<PythonSensorInput>(
                timestamp_ns, depth, labels, rgb, pipeline.sensor_name);
            input->sensor_input->input_feature = feature;
            return pipeline.step(input);
          },
          "timestamp_ns"_a,
          "world_t_body"_a,
          "world_R_body"_a,
          "depth"_a,
          "labels"_a,
          "rgb"_a,
          "feature"_a = FeatureVector())
      .def(
          "step",
          [](PythonPipeline& pipeline,
             size_t timestamp_ns,
             const Eigen::Vector3d& world_t_body,
             const Eigen::Vector4d& world_R_body,
             const PythonSensorInput::PointVec& points,
             const PythonSensorInput::LabelVec& labels,
             const PythonSensorInput::ColorVec& colors,
             const FeatureVector& feature) {
            auto input = std::make_shared<InputPacket>();
            input->timestamp_ns = timestamp_ns;
            input->world_t_body = world_t_body;
            input->world_R_body = Eigen::Quaterniond(
                world_R_body[0], world_R_body[1], world_R_body[2], world_R_body[3]);
            input->sensor_input = std::make_unique<PythonSensorInput>(
                timestamp_ns, points, labels, colors, pipeline.sensor_name);
            input->sensor_input->input_feature = feature;
            return pipeline.step(input);
          },
          "timestamp_ns"_a,
          "world_t_body"_a,
          "world_R_body"_a,
          "points"_a,
          "labels"_a,
          "colors"_a,
          "feature"_a = FeatureVector())
      .def("start", &PythonPipeline::start)
      .def("stop", &PythonPipeline::stop)
      .def_property_readonly("graph", &PythonPipeline::getSceneGraph);

  // Producer hook for offline DBoW: pushes a BowQuery onto bow_queue so the
  // frontend's assignBowVectors can stamp dbow_ids/dbow_values onto agent
  // attributes. The queue is allocated only when
  // GraphBuilder::Config::lcd_use_bow_vectors is true; this throws otherwise
  // to fail loudly instead of silently dropping the message.
  m.def(
      "push_bow_query",
      [](uint32_t robot_id,
         uint32_t pose_id,
         const std::vector<uint32_t>& word_ids,
         const std::vector<float>& word_values) {
        auto& queue = PipelineQueues::instance().bow_queue;
        if (!queue) {
          throw std::runtime_error(
              "bow_queue not allocated; set frontend.lcd_use_bow_vectors: true "
              "and ensure the pipeline is constructed before push_bow_query");
        }
        auto msg = std::make_shared<pose_graph_tools::BowQuery>();
        msg->robot_id = robot_id;
        msg->pose_id = pose_id;
        msg->bow_vector.word_ids = word_ids;
        msg->bow_vector.word_values = word_values;
        queue->push(msg);
      },
      "robot_id"_a,
      "pose_id"_a,
      "word_ids"_a,
      "word_values"_a);
}

}  // namespace python_pipeline

}  // namespace hydra::python

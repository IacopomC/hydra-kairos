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

/**
 * @file input_module.h
 * @brief The pipeline's front door. Runs a thread that drains the per-sensor
 *        data receivers, looks up the body pose at each frame's timestamp, packs
 *        the two into an InputPacket, and pushes it onto the output queue for
 *        the reconstruction stage.
 */

#include <config_utilities/virtual_config.h>

#include <atomic>
#include <thread>

#include "hydra/common/message_queue.h"
#include "hydra/common/module.h"
#include "hydra/input/data_receiver.h"
#include "hydra/input/input_packet.h"

namespace hydra {

/// @brief A queried relative pose: validity plus the target-from-source rotation
/// and translation, with helpers to test validity and build the transform.
struct PoseStatus {
  bool is_valid = false;
  Eigen::Quaterniond target_R_source;
  Eigen::Vector3d target_p_source;
  operator bool() const { return is_valid; }
  Eigen::Isometry3d target_T_source() const {
    return Eigen::Translation<double, 3>(target_p_source) * target_R_source;
  }
};

/// @brief Collects data from the configured sensor receivers, pairs each frame
/// with the body pose at its timestamp, and emits InputPackets. Subclasses
/// supply the pose lookup via getBodyPose().
class InputModule : public Module {
 public:
  using OutputQueue = MessageQueue<InputPacket::Ptr>;
  struct Config {
    /// @brief One named input: a data receiver paired with its sensor model.
    struct InputPair {
      config::VirtualConfig<DataReceiver> receiver;
      config::VirtualConfig<Sensor> sensor;
    };
    std::map<std::string, InputPair> inputs;  ///< Configured inputs, keyed by name.
  } const config;

  InputModule(const Config& config, const OutputQueue::Ptr& output_queue);

  virtual ~InputModule();

  void start() override;

  void stop() override;

  void save(const LogSetup& log_setup) override;

  std::string printInfo() const override;

 protected:
  /// @brief Worker loop: drain receivers, attach poses, emit InputPackets.
  void dataSpin();

  void stopImpl();

  /// @brief Look up the body pose in the world frame at @p timestamp.
  virtual PoseStatus getBodyPose(uint64_t timestamp) = 0;

 protected:
  OutputQueue::Ptr queue_;
  std::atomic<bool> should_shutdown_{false};

  std::vector<std::unique_ptr<DataReceiver>> receivers_;
  std::unique_ptr<std::thread> data_thread_;
};

void declare_config(InputModule::Config::InputPair& config);
void declare_config(InputModule::Config& config);

}  // namespace hydra

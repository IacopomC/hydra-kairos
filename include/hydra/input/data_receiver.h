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
 * @file data_receiver.h
 * @brief Base class for a per-sensor input source. Buffers incoming sensor
 *        packets on a queue for the input module to drain, and rate-limits them
 *        by enforcing a minimum separation between accepted timestamps.
 */

#include <optional>

#include "hydra/common/message_queue.h"
#include "hydra/input/sensor_input_packet.h"

namespace hydra {

/// @brief Receives sensor packets and queues them for the input module.
/// Subclasses implement the transport-specific setup in initImpl().
class DataReceiver {
 public:
  using DataQueue = MessageQueue<SensorInputPacket::Ptr>;

  struct Config {
    double input_separation_s = 0.0;  ///< Minimum spacing between accepted frames (s); 0 disables.
  };

  DataReceiver(const Config& config, const std::string& sensor_name);
  virtual ~DataReceiver() = default;

  /// @brief Initialize the receiver; returns false on failure.
  bool init();

 public:
  const Config config;
  DataQueue queue;  ///< Queue of received packets awaiting consumption.

 protected:
  /// @brief Transport-specific initialization, supplied by the subclass.
  virtual bool initImpl() = 0;

  /// @brief Accept or reject a frame by its timestamp per input_separation_s.
  bool checkInputTimestamp(uint64_t timestamp_ns);

  std::optional<uint64_t> last_time_received_;  ///< Timestamp of the last accepted frame.

  const std::string sensor_name_;  ///< Name of the sensor this receiver serves.
};

void declare_config(DataReceiver::Config& config);

}  // namespace hydra

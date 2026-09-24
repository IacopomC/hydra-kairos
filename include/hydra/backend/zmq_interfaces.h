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
#include <map>
#include <memory>
#include <mutex>
#include <thread>

#include "hydra/backend/backend_module.h"

/**
 * @file zmq_interfaces.h
 * @brief Backend components that exchange data over ZeroMQ. One sink publishes
 *        the optimized scene graph (and optionally its mesh) to subscribers; one
 *        update functor receives externally assigned room labels and applies
 *        them to the rooms layer.
 */

namespace spark_dsg {
class ZmqReceiver;
class ZmqSender;
}  // namespace spark_dsg

namespace hydra {

/// @brief Output sink that publishes the optimized scene graph over ZeroMQ.
class ZmqSink : public BackendModule::Sink {
 public:
  struct Config {
    std::string url = "tcp://127.0.0.1:8001";  ///< Address the graph is published on.
    bool send_mesh = true;                     ///< Whether the mesh is included in the published graph.
    size_t num_threads = 2;                    ///< Worker threads used by the publisher.
  } const config;

  /// @brief Constructs the sink and opens the publishing socket.
  explicit ZmqSink(const Config& config);
  virtual ~ZmqSink();
  /// @brief Publishes the current scene graph to subscribers.
  /// @param timestamp_ns Timestamp of the graph (ns).
  /// @param graph Scene graph to publish.
  /// @param dgraph Deformation graph accompanying the scene graph.
  void call(uint64_t timestamp_ns,
            const DynamicSceneGraph& graph,
            const kimera_pgmo::DeformationGraph& dgraph) const override;
  /// @brief Returns a human-readable description of the sink.
  std::string printInfo() const override;

 private:
  std::unique_ptr<spark_dsg::ZmqSender> sender_;  ///< Underlying ZeroMQ publisher.
};

/// @brief Declares the configuration fields for ZmqSink.
void declare_config(ZmqSink::Config& config);

/// @brief Update functor that applies externally received room labels over ZeroMQ.
class ZmqRoomLabelUpdater : public UpdateFunctor {
 public:
  struct Config {
    std::string url = "tcp://127.0.0.1:8002";  ///< Address room labels are received on.
    size_t num_threads = 2;                    ///< Worker threads used by the receiver.
    size_t poll_time_ms = 10;                  ///< Receiver poll interval (ms).
  } const config;

  /// @brief Constructs the updater and starts the background receive thread.
  ZmqRoomLabelUpdater(const Config& config);
  virtual ~ZmqRoomLabelUpdater();
  /// @brief Applies the most recently received room labels to the rooms layer.
  /// @param graph Shared graph whose room names are updated.
  void call(const DynamicSceneGraph&,
            SharedDsgInfo& graph,
            const UpdateInfo::ConstPtr&) const override;

 private:
  /// @brief Drains pending label messages from the receiver into the name map.
  void checkForUpdates();

  mutable std::mutex mutex_;                    ///< Guards the room-name map against the receive thread.
  std::unique_ptr<std::thread> thread_;         ///< Background thread polling for incoming labels.
  std::atomic<bool> should_shutdown_{false};    ///< Signals the receive thread to stop.
  std::unique_ptr<spark_dsg::ZmqReceiver> receiver_;  ///< Underlying ZeroMQ subscriber.
  std::map<NodeId, std::string> room_name_map_;  ///< Latest received label for each room node.
};

/// @brief Declares the configuration fields for ZmqRoomLabelUpdater.
void declare_config(ZmqRoomLabelUpdater::Config& conf);

}  // namespace hydra

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
#include <memory>
#include <queue>
#include <thread>
#include <unordered_map>

#include "hydra/common/common_types.h"
#include "hydra/common/module.h"
#include "hydra/common/shared_dsg_info.h"
#include "hydra/common/shared_module_state.h"
#include "hydra/loop_closure/detector.h"
#include "hydra/loop_closure/loop_closure_config.h"
#include "hydra/utils/log_utilities.h"

/**
 * @file loop_closure_module.h
 * @brief Module that runs loop-closure detection on its own thread. It drains
 *        frontend updates, maintains a local copy of the scene graph, drives
 *        the detector against each query agent, and forwards the resulting
 *        constraints to the backend.
 */
namespace hydra {

/// @brief Threaded module wrapping the loop-closure detector.
///
/// Each spin merges the latest frontend graph into a local copy, picks a query
/// agent within the configured time horizon, updates the descriptor caches, and
/// runs detection. Valid solutions are deduplicated per anchor and pushed to
/// the backend queue.
class LoopClosureModule : public Module {
 public:
  /// @brief Construct the module.
  /// @param config Module and detector configuration.
  /// @param state Shared pipeline state, including the scene graph the module reads.
  LoopClosureModule(const LoopClosureConfig& config,
                    const SharedModuleState::Ptr& state);

  virtual ~LoopClosureModule();

  /// @brief Launch the spin thread.
  void start() override;

  /// @brief Signal the spin thread to stop and join it.
  void stop() override;

  /// @brief Write any module output for the run to the log location.
  /// @param log_setup Logging paths for the run.
  void save(const LogSetup& log_setup) override;

  /// @brief Summarize module state for logging.
  /// @return A human-readable status string.
  std::string printInfo() const override;

  /// @brief Spin loop body run on the worker thread until shutdown.
  void spin();

  /// @brief Process at most one queued update.
  /// @param force_update Run detection even when the graph sequence number is unchanged.
  /// @return True when an update was processed, false when no data was waiting.
  bool spinOnce(bool force_update);

  /// @brief Access the underlying detector.
  /// @return Reference to the detector.
  lcd::LcdDetector& getDetector() const;

 protected:
  /// @brief Merge the latest graph, run detection, and forward results.
  /// @param force_update Run detection even when the graph sequence number is unchanged.
  void spinOnceImpl(bool force_update);

  /// @brief Drain queued frontend inputs into the module's working state.
  /// @return Timestamp (ns) of the most recent input drained.
  size_t processFrontendOutput();

  /// @brief Select the place nodes near the query agent to describe and cache.
  /// @param agent_pos World position of the query agent.
  /// @return The set of nearby place node identifiers.
  NodeIdSet getPlacesToCache(const Eigen::Vector3d& agent_pos);

  /// @brief Choose the agent node to query once it is past the time horizon.
  /// @param timestamp_ns Current timestamp in nanoseconds.
  /// @return The query agent identifier, or empty when no agent is ready.
  std::optional<NodeId> getQueryAgentId(size_t timestamp_ns);

 protected:
  std::atomic<bool> should_shutdown_{false};       ///< Set to break the spin loop.
  std::unique_ptr<std::thread> spin_thread_;       ///< Worker thread running spin().
  uint64_t last_sequence_number_ = 0;              ///< Sequence number of the last processed graph state.

  LoopClosureConfig config_;        ///< Module and detector configuration.
  SharedModuleState::Ptr state_;    ///< Shared pipeline state.

  std::priority_queue<NodeId, std::vector<NodeId>, std::greater<NodeId>> agent_queue_;  ///< Pending agent nodes, smallest identifier first.
  std::list<NodeId> potential_lcd_root_nodes_;  ///< Place nodes awaiting descriptor creation.

  std::unique_ptr<lcd::LcdDetector> lcd_detector_;  ///< The loop-closure detector.
  DynamicSceneGraph::Ptr lcd_graph_;                ///< Local working copy of the scene graph.

  /// Maps a closure target node to the timestamp (ns) it last fired at, so a
  /// result whose target fired recently is dropped before reaching the backend.
  /// This collapses a burst of revisit matches into one closure per anchor
  /// regardless of which layer produced them; the window comes from
  /// config_.bow_agent.dedup_window_s.
  std::unordered_map<NodeId, uint64_t> last_fired_against_;

 private:
  /// @brief Stop the worker thread; shared by stop() and the destructor.
  void stopImpl();
};

}  // namespace hydra

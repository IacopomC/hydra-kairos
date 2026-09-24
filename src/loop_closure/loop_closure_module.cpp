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
#include "hydra/loop_closure/loop_closure_module.h"

#include <config_utilities/printing.h>
#include <glog/logging.h>
#include <kimera_pgmo/utils/common_functions.h>
#include <spark_dsg/printing.h>

#include <algorithm>

#include "hydra/common/global_info.h"
#include "hydra/common/pipeline_queues.h"
#include "hydra/loop_closure/lcd_input.h"
#include "hydra/utils/timing_utilities.h"

namespace hydra {

using hydra::timing::ScopedTimer;
using lcd::LayerRegistrationConfig;

LoopClosureModule::LoopClosureModule(const LoopClosureConfig& config,
                                     const SharedModuleState::Ptr& state)
    : config_(config), state_(state), lcd_graph_(new DynamicSceneGraph()) {
  lcd_detector_.reset(new lcd::LcdDetector(config_.detector));
}

LoopClosureModule::~LoopClosureModule() { stopImpl(); }

void LoopClosureModule::start() {
  spin_thread_.reset(new std::thread(&LoopClosureModule::spin, this));
  LOG(INFO) << "[Hydra LCD] LCD started!";
}

void LoopClosureModule::stop() { stopImpl(); }

void LoopClosureModule::stopImpl() {
  VLOG(2) << "[Hydra LCD] stopping lcd!";

  should_shutdown_ = true;
  if (spin_thread_) {
    VLOG(2) << "[Hydra LCD] joining thread";
    spin_thread_->join();
    spin_thread_.reset();
    VLOG(2) << "[Hydra LCD] joined thread";
  }
}

void LoopClosureModule::save(const LogSetup& log_setup) {
  const auto log_path = log_setup.getLogDir("lcd");
  lcd_detector_->dumpDescriptors(log_path);
  lcd_graph_->save(log_path + "/dsg.json", false);
}

std::string LoopClosureModule::printInfo() const { return config::toString(config_); }

void LoopClosureModule::spin() {
  auto queue = PipelineQueues::instance().lcd_queue;
  if (!queue) {
    LOG(ERROR) << "LCD queue required to run LCD";
    return;
  }

  bool should_shutdown = false;
  while (!should_shutdown) {
    bool has_data = queue->poll();
    if (GlobalInfo::instance().force_shutdown() || !has_data) {
      // copy over shutdown request
      should_shutdown = should_shutdown_;
    }

    if (!has_data) {
      continue;
    }

    // TODO: consider config option for this
    // for now, we only update the lcd graph when the latest popped message
    // has a timestamp after the graph update (and only do lcd after an update)
    // this is okay as the frontend pushes output before marking the last update time
    // i.e.: lcd will always either be able to update the lcd graph copy or be able to
    // pop another message from the queue) This could potentially starve running LCD
    // when popping frontend outputs takes longer than it takes the frontend to
    // reproduce them (which is unlikely)
    spinOnceImpl(false);
  }
}

bool LoopClosureModule::spinOnce(bool force_update) {
  auto queue = PipelineQueues::instance().lcd_queue;
  if (!queue) {
    LOG(ERROR) << "LCD queue required to run LCD";
    return false;
  }

  bool has_data = queue->poll();
  if (!has_data) {
    return false;
  }

  spinOnceImpl(force_update);
  return true;
}

lcd::LcdDetector& LoopClosureModule::getDetector() const { return *lcd_detector_; }

void LoopClosureModule::spinOnceImpl(bool force_update) {
  const size_t timestamp_ns = processFrontendOutput();

  const auto& dsg = *state_->lcd_graph;
  {  // start critical section
    std::unique_lock<std::mutex> lock(dsg.mutex);
    if (!force_update && last_sequence_number_ != dsg.sequence_number) {
      return;
    }

    ScopedTimer spin_timer("lcd/merge_graph", timestamp_ns);
    lcd_graph_->mergeGraph(*dsg.graph);
  }  // end critical section

  // Process every agent that is now old enough to query. For each one: refresh
  // the descriptor cache with newly archived places near it, run detection,
  // and forward any surviving loop closures to the backend.
  auto query_agent = getQueryAgentId(timestamp_ns);
  while (query_agent) {
    // Cache descriptors for places that have settled near this agent before
    // matching against them.
    const Eigen::Vector3d query_pos = lcd_graph_->getPosition(*query_agent);
    const auto to_cache = getPlacesToCache(query_pos);

    if (!to_cache.empty()) {
      VLOG(5) << "[Hydra LCD] Constructing descriptors for "
              << displayNodeSymbolContainer(to_cache);
      lcd_detector_->updateDescriptorCache(*lcd_graph_, to_cache, timestamp_ns);
    }

    const auto time = lcd_graph_->getNode(*query_agent).timestamp.value();
    auto results = lcd_detector_->detect(*lcd_graph_, *query_agent, time.count());
    auto& queue = PipelineQueues::instance().backend_lcd_queue;
    const auto dedup_window_ns = static_cast<uint64_t>(
        std::max(0.0, config_.bow_agent.dedup_window_s) * 1e9);
    for (const auto& result : results) {
      // Cross-call dedup against the same match anchor. Whichever layer
      // produced this LC, if we already fired against `to_node` recently we
      // drop the result. This keeps revisit storms (many consecutive query
      // agents all matching the same anchor) from cascading into the backend
      // as one loop closure each. The window is the same one BowAgentSolver
      // uses for layer-0 dedup; the two mechanisms compose harmlessly.
      const uint64_t now_ns = static_cast<uint64_t>(time.count());
      auto last_iter = last_fired_against_.find(result.to_node);
      if (dedup_window_ns > 0 && last_iter != last_fired_against_.end() &&
          now_ns >= last_iter->second &&
          (now_ns - last_iter->second) < dedup_window_ns) {
        LOG(INFO) << "[Hydra LCD] dedup-suppressed loop-closure: "
                  << NodeSymbol(result.from_node).getLabel() << " -> "
                  << NodeSymbol(result.to_node).getLabel() << " (level="
                  << result.level << ", last fired "
                  << (now_ns - last_iter->second) * 1e-9 << "s ago)";
        continue;
      }
      last_fired_against_[result.to_node] = now_ns;

      // TODO: consider augmenting with gtsam key
      queue.push(result);
      LOG(WARNING) << "[Hydra LCD] Found valid loop-closure: "
                   << NodeSymbol(result.from_node).getLabel() << " -> "
                   << NodeSymbol(result.to_node).getLabel()
                   << " (level=" << result.level << ")";
    }

    // if should_shutdown_ is true and agent/place parent invariant is broken, this
    // will exit early
    query_agent = getQueryAgentId(timestamp_ns);
  }
}

size_t LoopClosureModule::processFrontendOutput() {
  auto queue = PipelineQueues::instance().lcd_queue;
  const auto& msg = queue->front();
  VLOG(5) << "[Hydra LCD] Received archived places: "
          << displayNodeSymbolContainer(msg->archived_places);

  potential_lcd_root_nodes_.insert(potential_lcd_root_nodes_.end(),
                                   msg->archived_places.begin(),
                                   msg->archived_places.end());

  VLOG(5) << "[Hydra LCD] Adding nodes: "
          << displayNodeSymbolContainer(msg->new_agent_nodes);
  for (const auto& node : msg->new_agent_nodes) {
    agent_queue_.push(node);
  }

  size_t timestamp_ns = msg->timestamp_ns;
  last_sequence_number_ = msg->sequence_number;
  queue->pop();
  return timestamp_ns;
}

NodeIdSet LoopClosureModule::getPlacesToCache(const Eigen::Vector3d& agent_pos) {
  NodeIdSet to_cache;
  size_t deleted_count = 0;
  size_t within_horizon_count = 0;
  size_t reactivated_count = 0;
  auto iter = potential_lcd_root_nodes_.begin();
  while (iter != potential_lcd_root_nodes_.end()) {
    auto node_opt = lcd_graph_->findNode(*iter);
    if (!node_opt) {
      VLOG(5) << "[Hydra LCD] Deleted place " << NodeSymbol(*iter).getLabel()
              << " found in LCD queue";
      ++deleted_count;
      iter = potential_lcd_root_nodes_.erase(iter);
      continue;
    }

    const auto& attrs = node_opt->attributes();
    if ((agent_pos - attrs.position).norm() < config_.descriptor_creation_horizon_m) {
      ++within_horizon_count;
      ++iter;
      continue;
    }

    // The places in this queue arrived via msg->archived_places, i.e. they
    // were is_active=false at the moment of archival. The frontend can flip
    // is_active back to true on revisit (gvd_place_extractor / mesh_segmenter
    // / place_2d_segmenter all do this). That happens routinely when the
    // robot revisits the same physical area, especially when
    // descriptor_creation_horizon_m << active_place_radius_m (small space,
    // small descriptor horizon). The previous CHECK aborted in that case;
    // instead, leave the place in the queue and retry on a later tick once
    // it has settled. Caching a still-active node would race with the
    // frontend's edits to its attributes, so skipping is also the right
    // semantic answer, not just a workaround.
    if (attrs.is_active) {
      ++reactivated_count;
      ++iter;
      continue;
    }

    to_cache.insert(*iter);
    iter = potential_lcd_root_nodes_.erase(iter);
  }

  // Summary diagnostic: tells us at a glance whether the queue is draining
  // (to_cache > 0) or piling up (queue_size growing while to_cache stays
  // empty). reactivated_count much larger than to_cache means the active-place
  // radius is much wider than descriptor_creation_horizon_m, so most of the
  // queue is being soft-skipped every tick.
  if (!potential_lcd_root_nodes_.empty() || !to_cache.empty()) {
    VLOG(2) << "[Hydra LCD] getPlacesToCache: queue=" << potential_lcd_root_nodes_.size()
            << " to_cache=" << to_cache.size()
            << " skipped(within_horizon)=" << within_horizon_count
            << " skipped(reactivated)=" << reactivated_count
            << " deleted=" << deleted_count;
  }

  return to_cache;
}

std::optional<NodeId> LoopClosureModule::getQueryAgentId(size_t stamp_ns) {
  if (agent_queue_.empty()) {
    return std::nullopt;
  }

  const auto& node = lcd_graph_->getNode(agent_queue_.top());
  const auto prev_time = node.timestamp.value();

  if (!node.hasParent()) {
    LOG(ERROR) << "Found agent node without parent: "
               << NodeSymbol(agent_queue_.top()).getLabel() << ". Discarding!";
    agent_queue_.pop();
    return std::nullopt;
  }

  std::chrono::duration<double> diff_s = std::chrono::nanoseconds(stamp_ns) - prev_time;
  // we consider should_shutdown_ here to make sure we're not waiting on popping from
  // the LCD queue while not getting new place messages
  if (!should_shutdown_ && diff_s.count() < config_.lcd_agent_horizon_s) {
    return std::nullopt;
  }

  if (should_shutdown_ && (diff_s.count() < config_.lcd_agent_horizon_s)) {
    LOG(ERROR) << "Forcing pop of node " << NodeSymbol(agent_queue_.top()).getLabel()
               << " from lcd queue due to shutdown: "
               << ", diff: " << diff_s.count() << " / " << config_.lcd_agent_horizon_s;
  }

  auto valid_node = agent_queue_.top();
  agent_queue_.pop();
  return valid_node;
}

}  // namespace hydra

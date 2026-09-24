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
#include <spark_dsg/dynamic_scene_graph.h>

#include <atomic>
#include <map>
#include <memory>
#include <mutex>

/**
 * @file shared_dsg_info.h
 * @brief Scene graph shared between pipeline modules along with the
 *        synchronization and bookkeeping needed to share it: a guarding mutex,
 *        an update flag and sequence number, the layer/prefix mappings, and
 *        pending node merges.
 */

namespace hydra {

/// @brief A scene graph plus the state needed to share it across modules safely.
struct SharedDsgInfo {
  using Ptr = std::shared_ptr<SharedDsgInfo>;

  /// @brief Construct an info object with an empty graph and the given mappings.
  /// @param layer_id_map Maps each layer to the symbol prefix used for its nodes.
  explicit SharedDsgInfo(const std::map<spark_dsg::LayerId, char>& layer_id_map);

  /// @brief Produce a deep copy of this shared info.
  SharedDsgInfo::Ptr clone() const;

  // Mutexes are ordered to avoid deadlock:
  //   1. SharedDsgInfo::mutex (loop-closure detection)
  //   2. SharedDsgInfo::mutex (backend)
  //   3. SharedDsgInfo::mutex (frontend)
  //   4. SharedModuleState::mesh_mutex
  // When acquiring two of these, always lock the lower-numbered one first.
  mutable std::mutex mutex;            ///< Guards access to this graph; see the ordering note above.
  std::atomic<bool> updated;           ///< Set when the graph has changed since last consumed.
  uint64_t sequence_number = 0;        ///< Monotonic counter incremented on update.
  spark_dsg::DynamicSceneGraph::Ptr graph;  ///< The shared scene graph.
  std::map<char, spark_dsg::LayerId> prefix_layer_map;  ///< Layer for each symbol prefix.
  std::map<spark_dsg::LayerId, char> layer_prefix_map;  ///< Symbol prefix for each layer.
  std::map<spark_dsg::NodeId, spark_dsg::NodeId> merges;  ///< Pending node merges (from -> into).

 protected:
  SharedDsgInfo();
};

}  // namespace hydra

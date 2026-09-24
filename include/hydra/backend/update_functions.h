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
#include <gtsam/nonlinear/Values.h>

#include "hydra/backend/merge_proposer.h"
#include "hydra/common/dsg_types.h"
#include "hydra/common/shared_dsg_info.h"

/**
 * @file update_functions.h
 * @brief Base interface for the per-layer update functors the backend runs
 *        after optimization. Defines the per-iteration context shared with the
 *        functors and the optional cleanup and attribute-merge hooks they can
 *        expose.
 */

namespace kimera_pgmo {
class DeformationGraph;
}

namespace hydra {

/// @brief Per-iteration context passed to every update functor.
struct UpdateInfo {
  using Ptr = std::shared_ptr<UpdateInfo>;
  using ConstPtr = std::shared_ptr<const UpdateInfo>;
  /// @brief Externally supplied merges grouped by the layer they apply to.
  using LayerMerges = std::map<LayerId, MergeList>;

  const gtsam::Values* places_values = nullptr;  ///< Optimized poses for the places layer, if available.
  const gtsam::Values* pgmo_values = nullptr;    ///< Optimized mesh/agent poses from the optimizer.
  bool loop_closure_detected = false;            ///< Whether a loop closure triggered this iteration.
  uint64_t timestamp_ns = 0;                     ///< Timestamp of this iteration (ns).
  LayerMerges given_merges;                      ///< Externally supplied merges to apply this iteration.
  const gtsam::Values* complete_agent_values = nullptr;  ///< Optimized poses for all agent nodes.
  kimera_pgmo::DeformationGraph* deformation_graph = nullptr;  ///< Deformation graph for non-optimized interpolation.
  const std::unordered_map<NodeId, size_t>* node_to_robot_id = nullptr;  ///< Maps a node to the robot that owns it.
};

/// @brief Callback that performs post-update cleanup on a layer.
using LayerCleanupFunc =
    std::function<void(const UpdateInfo::ConstPtr&, SharedDsgInfo*)>;
/// @brief Callback that synthesizes the attributes of a merged node from its members.
using MergeFunc = std::function<NodeAttributes::Ptr(const DynamicSceneGraph&,
                                                    const std::vector<NodeId>&)>;

/// @brief Base class for the per-layer update steps the backend runs after optimization.
struct UpdateFunctor {
  using Ptr = std::shared_ptr<UpdateFunctor>;

  /// @brief Optional callbacks a functor exposes to the backend.
  struct Hooks {
    LayerCleanupFunc cleanup;  ///< Cleanup callback, empty if unused.
    MergeFunc merge;           ///< Attribute-merge callback, empty if unused.
  };

  virtual ~UpdateFunctor() = default;
  /// @brief Returns the functor's hooks; the base implementation returns none.
  virtual Hooks hooks() const;
  /// @brief Applies this functor's update to the graph.
  /// @param unmerged Graph holding the original, unmerged nodes.
  /// @param dsg Shared graph to update.
  /// @param info Per-iteration context.
  virtual void call(const DynamicSceneGraph& unmerged,
                    SharedDsgInfo& dsg,
                    const UpdateInfo::ConstPtr& info) const = 0;
  /// @brief Proposes merges for this functor's layer; the base returns none.
  /// @param graph Graph to search for merges.
  /// @param info Per-iteration context.
  /// @return Proposed merges.
  virtual MergeList findMerges(const DynamicSceneGraph& graph,
                               const UpdateInfo::ConstPtr& info) const;
};

}  // namespace hydra

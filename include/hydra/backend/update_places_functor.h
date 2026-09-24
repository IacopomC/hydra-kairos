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
#include <config_utilities/factory.h>

#include "hydra/backend/association_strategies.h"
#include "hydra/backend/merge_tracker.h"
#include "hydra/backend/update_functions.h"
#include "hydra/utils/active_window_tracker.h"

/**
 * @file update_places_functor.h
 * @brief Update functor for the places layer. Moves place nodes to their
 *        optimized positions when the optimizer produced values for them, or
 *        otherwise deforms them by interpolating nearby control points, and
 *        proposes merges between nearby places of comparable size.
 */

namespace hydra {

/// @brief Updates place-layer positions and proposes place merges.
struct UpdatePlacesFunctor : public UpdateFunctor {
  struct Config {
    double pos_threshold_m = 0.4;        ///< Max centroid distance for a place merge to be considered.
    double distance_tolerance_m = 0.4;   ///< Max difference in place radius for a merge to be considered.
    size_t num_control_points = 4;       ///< Control points used to deform places outside optimization.
    double control_point_tolerance_s = 10.0;  ///< Timestamp tolerance when selecting deformation control points.
    /// @brief Propose merges over the full layer every N updates (0 disables).
    ///
    /// Merge proposals normally originate only from nodes in the active window, so
    /// two already-archived places can never merge with each other. A periodic
    /// full-layer pass lets archived duplicates (e.g. from revisits) collapse.
    size_t full_merge_every_n_updates = 0;
    /// @brief Association strategy used to find place merge candidates.
    MergeProposer::Config merge_proposer = {
        config::VirtualConfig<AssociationStrategy>{association::NearestNode::Config{}}};
  } const config;

  /// @brief Constructs the functor from a validated configuration.
  explicit UpdatePlacesFunctor(const Config& config);
  /// @brief Updates place positions, either from optimized values or by interpolation.
  /// @param unmerged Graph holding the original, unmerged nodes.
  /// @param dsg Shared graph whose places are updated.
  /// @param info Per-iteration context.
  void call(const DynamicSceneGraph& unmerged,
            SharedDsgInfo& dsg,
            const UpdateInfo::ConstPtr& info) const override;

  /// @brief Moves places to positions taken directly from the optimized values.
  /// @return Number of places updated.
  size_t updateFromValues(const LayerView& view,
                          SharedDsgInfo& dsg,
                          const UpdateInfo::ConstPtr& info) const;

  /// @brief Deforms places by interpolating nearby control points when no optimized value exists.
  /// @return Number of places updated.
  size_t interpFromValues(const LayerView& view,
                          SharedDsgInfo& dsg,
                          const UpdateInfo::ConstPtr& info) const;

  /// @brief Proposes merges between nearby places of comparable size.
  /// @param graph Graph to search for place merges.
  /// @param info Per-iteration context.
  /// @return Proposed place merges.
  MergeList findMerges(const DynamicSceneGraph& graph,
                       const UpdateInfo::ConstPtr& info) const override;

  /// @brief Returns the best merge target for a place, if any candidate qualifies.
  /// @param layer Places layer holding the candidates.
  /// @param node Place to find a merge target for.
  /// @return Target node id, or empty if no candidate qualifies.
  std::optional<NodeId> proposeMerge(const SceneGraphLayer& layer,
                                     const SceneGraphNode& node) const;

  /// @brief Removes places that lack an optimized position from the graph.
  /// @param graph Graph to prune.
  /// @param missing_nodes Places without an optimized value.
  void filterMissing(DynamicSceneGraph& graph,
                     const std::list<NodeId> missing_nodes) const;

  mutable ActiveWindowTracker active_tracker;  ///< Tracks which places are in the active window.
  mutable std::unordered_map<NodeId, Eigen::Vector3d> cached_pos_;  ///< Last known position per place, for interpolation.
  const MergeProposer merge_proposer;          ///< Proposer that selects place merge candidates.
  mutable size_t updates_since_full_merge_ = 0;  ///< Updates elapsed since the last full-layer merge pass.
  mutable bool full_merge_pending_ = false;      ///< Whether findMerges should scan the full layer this update.

 private:
  inline static const auto registration_ =
      config::RegistrationWithConfig<UpdateFunctor, UpdatePlacesFunctor, Config>(
          "UpdatePlacesFunctor");
};

/// @brief Declares the configuration fields for UpdatePlacesFunctor.
void declare_config(UpdatePlacesFunctor::Config& config);

}  // namespace hydra

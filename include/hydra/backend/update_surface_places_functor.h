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
#include "hydra/backend/update_functions.h"
#include "hydra/utils/active_window_tracker.h"

/**
 * @file update_surface_places_functor.h
 * @brief Update functor for the surface-place (2D place) layer. Refreshes each
 *        surface place from the optimized mesh, proposes merges between nearby
 *        co-planar places, and optionally splits oversized places into smaller
 *        ones during cleanup.
 */

namespace hydra {

/// @brief Updates, merges, and optionally splits surface-place (2D place) nodes.
struct Update2dPlacesFunctor : public UpdateFunctor {
  struct Config {
    bool allow_places_merge = true;        ///< Whether nearby surface places may be merged.
    double merge_max_delta_z = 0.5;        ///< Two places differing by at least this much in z are not merged.
    size_t min_points = 10;                ///< Minimum vertex count before a place may be split.
    double min_size = 2;                   ///< Minimum spatial extent before a place may be split.
    double connection_overlap_threshold = 0;  ///< Overlap between places required to add a connecting edge.
    double connection_max_delta_z = 0.5;   ///< Maximum z separation between connected neighboring places.
    double connection_ellipse_scale_factor = 1.0;  ///< Inflation of the place ellipse relative to its bounding box.
    bool enable_splitting = false;         ///< Whether oversized surface places are split during cleanup.
    /// @brief Propose merges over the full layer every N updates (0 disables).
    ///
    /// Merge proposals normally originate only from nodes in the active window, so
    /// two already-archived surface places can never merge with each other. A
    /// periodic full-layer pass lets archived duplicates (e.g. from revisits)
    /// collapse.
    size_t full_merge_every_n_updates = 0;
    /// @brief Association strategy used to find surface-place merge candidates.
    MergeProposer::Config merge_proposer = {config::VirtualConfig<AssociationStrategy>{
        association::SemanticNearestNode::Config{}}};
  } const config;

  /// @brief Constructs the functor from a validated configuration.
  explicit Update2dPlacesFunctor(const Config& config);
  /// @brief Exposes the cleanup and merge hooks for surface places.
  Hooks hooks() const override;
  /// @brief Refreshes surface-place geometry from the optimized mesh.
  /// @param unmerged Graph holding the original, unmerged nodes.
  /// @param dsg Shared graph whose surface places are updated.
  /// @param info Per-iteration context.
  void call(const DynamicSceneGraph& unmerged,
            SharedDsgInfo& dsg,
            const UpdateInfo::ConstPtr& info) const override;
  /// @brief Proposes merges between nearby co-planar surface places.
  /// @param graph Graph to search for merges.
  /// @param info Per-iteration context.
  /// @return Proposed surface-place merges.
  MergeList findMerges(const DynamicSceneGraph& graph,
                       const UpdateInfo::ConstPtr& info) const override;

  /// @brief Recomputes a single surface place's attributes from the mesh.
  /// @param mesh Optimized mesh providing the vertex positions.
  /// @param node Surface place to update.
  /// @param attrs Attributes of the surface place, updated in place.
  void updateNode(const spark_dsg::Mesh::Ptr& mesh,
                  NodeId node,
                  Place2dNodeAttributes& attrs) const;

  /// @brief Decides whether one surface place may merge into another.
  /// @param from_attrs Attributes of the place being absorbed.
  /// @param to_attrs Attributes of the surviving place.
  /// @return True if the two places should merge.
  bool shouldMerge(const Place2dNodeAttributes& from_attrs,
                   const Place2dNodeAttributes& to_attrs) const;

  /// @brief Finishes pending merges and splits oversized surface places.
  /// @param dsg Shared graph whose surface places are cleaned up.
  void cleanup(SharedDsgInfo& dsg) const;

  mutable ActiveWindowTracker active_tracker;  ///< Tracks which surface places are in the active window.
  const MergeProposer merge_proposer;          ///< Proposer that selects surface-place merge candidates.
  mutable size_t updates_since_full_merge_ = 0;  ///< Updates elapsed since the last full-layer merge pass.
  mutable bool full_merge_pending_ = false;      ///< Whether findMerges should scan the full layer this update.

 private:
  mutable NodeSymbol next_node_id_ = NodeSymbol('S', 0);  ///< Next symbol assigned to places created by splitting.
  const LayerId layer_id_ = DsgLayers::MESH_PLACES;       ///< Layer this functor operates on.

  inline static const auto registration_ =
      config::RegistrationWithConfig<UpdateFunctor, Update2dPlacesFunctor, Config>(
          "Update2dPlacesFunctor");
};

/// @brief Declares the configuration fields for Update2dPlacesFunctor.
void declare_config(Update2dPlacesFunctor::Config& conf);

}  // namespace hydra

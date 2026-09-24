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
 * @file update_objects_functor.h
 * @brief Update functor for the objects layer. Tracks the active objects,
 *        proposes merges between objects via an association strategy, and
 *        combines the attributes (and optionally mesh connections) of merged
 *        objects.
 */

namespace hydra {

/// @brief Updates and merges object-layer nodes.
struct UpdateObjectsFunctor : public UpdateFunctor {
  struct Config {
    bool allow_connection_merging = true;  ///< Whether merged objects pool their mesh-vertex connections.
    /// @brief Association strategy used to find merge candidates among active objects.
    MergeProposer::Config merge_proposer = {config::VirtualConfig<AssociationStrategy>{
        association::SemanticNearestNode::Config{}}};
  } const config;

  /// @brief Constructs the functor from a validated configuration.
  explicit UpdateObjectsFunctor(const Config& config);
  /// @brief Exposes the merge hook that synthesizes merged object attributes.
  Hooks hooks() const override;
  /// @brief Updates the active object set for this iteration.
  /// @param unmerged Graph holding the original, unmerged nodes.
  /// @param dsg Shared graph whose objects are updated.
  /// @param info Per-iteration context.
  void call(const DynamicSceneGraph& unmerged,
            SharedDsgInfo& dsg,
            const UpdateInfo::ConstPtr& info) const override;
  /// @brief Proposes merges between active objects.
  /// @param graph Graph to search for object merges.
  /// @param info Per-iteration context.
  /// @return Proposed object merges.
  MergeList findMerges(const DynamicSceneGraph& graph,
                       const UpdateInfo::ConstPtr& info) const override;

  /// @brief Combines the attributes of two objects when one is merged into the other.
  /// @param layer Graph containing both objects.
  /// @param from Object being absorbed.
  /// @param to Object that survives.
  void mergeAttributes(const DynamicSceneGraph& layer, NodeId from, NodeId to) const;

  mutable ActiveWindowTracker active_tracker;  ///< Tracks which objects are in the active window.
  const MergeProposer merge_proposer;          ///< Proposer that selects object merge candidates.

 private:
  inline static const auto registration_ =
      config::RegistrationWithConfig<UpdateFunctor, UpdateObjectsFunctor, Config>(
          "UpdateObjectsFunctor");
};

/// @brief Declares the configuration fields for UpdateObjectsFunctor.
void declare_config(UpdateObjectsFunctor::Config& config);

}  // namespace hydra

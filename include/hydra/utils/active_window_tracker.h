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
#include <spark_dsg/layer_view.h>

#include "hydra/common/dsg_types.h"

/**
 * @file active_window_tracker.h
 * @brief Tracks which scene graph nodes fall inside the active window so they can be
 *        iterated as a layer view. A node stays in the window from the moment it
 *        becomes active until it has been archived and explicitly cleared, which keeps
 *        just-archived nodes visible for one more pass.
 */

namespace hydra {

/// @brief Exposes a filtered view over nodes that are currently active or were active
/// when last seen and have not yet been cleared.
struct ActiveWindowTracker {
  using Ptr = std::unique_ptr<ActiveWindowTracker>;  ///< Owning pointer alias.

  /// @brief Build a view over the active window of the given layer.
  /// The view filters to active nodes plus nodes that became inactive since the last
  /// call but have not been cleared. Removed nodes are pruned from internal state unless
  /// frozen.
  /// @param layer Layer to view.
  /// @param freeze If true, internal tracking state is left untouched by this call.
  /// @return A layer view restricted to the active window.
  spark_dsg::LayerView view(const SceneGraphLayer& layer, bool freeze = false) const;
  /// @brief Drop the just-archived nodes from the window so they no longer iterate.
  void clear();
  /// @brief Forget all tracked nodes and reset to an empty window.
  void reset();

 private:
  /// @brief Decide whether a node belongs in the active window, updating tracking state.
  bool isActive(const SceneGraphNode& node, bool freeze = false) const;
  mutable std::set<NodeId> to_clear_;     ///< Archived nodes still shown until cleared.
  mutable std::set<NodeId> prev_active_;  ///< Nodes seen active on a previous view call.
};

}  // namespace hydra

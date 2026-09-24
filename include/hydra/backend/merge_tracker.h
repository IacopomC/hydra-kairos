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
#include "hydra/backend/update_functions.h"

/**
 * @file merge_tracker.h
 * @brief Applies proposed merges to the shared graph and remembers the history
 *        of which nodes were folded together. The remap history lets later
 *        proposals that still reference an absorbed node be redirected to its
 *        surviving target, and the per-target child sets let merged attributes
 *        be recomputed from all original members.
 */

namespace hydra {

/// @brief Applies merge proposals and tracks the accumulated merge history.
struct MergeTracker {
  /// @brief Applies the given merges to the graph and refreshes merged attributes.
  /// Endpoints are first remapped through the prior-merge history; stale and
  /// already-applied proposals are skipped. The history and per-target child
  /// sets are pruned of nodes that no longer exist in the graph.
  /// @param unmerged Graph holding the original, unmerged nodes used to recompute attributes.
  /// @param proposals Merges to apply this iteration.
  /// @param dsg Shared graph that receives the merges and stores the remap history.
  /// @param merge_attrs Optional callback recomputing attributes from the original members.
  /// @return Number of merges actually applied.
  size_t applyMerges(const DynamicSceneGraph& unmerged,
                     const MergeList& proposals,
                     SharedDsgInfo& dsg,
                     const MergeFunc& merge_attrs = MergeFunc());

  /// @brief Clears the accumulated merge history.
  void clear();

 private:
  /// @brief Redirects existing history entries that targeted the absorbed node
  /// to the surviving node after a new merge is applied.
  void updateParents(std::map<NodeId, NodeId>& prior_merges, const Merge& merge);

  std::map<NodeId, std::set<NodeId>> merge_sets_;  ///< Original member nodes folded into each surviving node.
};

}  // namespace hydra

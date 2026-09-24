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

#include <config_utilities/virtual_config.h>
#include <spark_dsg/layer_view.h>

#include "hydra/common/dsg_types.h"

/**
 * @file merge_proposer.h
 * @brief Proposes node merges within a scene-graph layer. Defines the merge
 *        record, the association-strategy interface that selects candidate
 *        nodes, and the proposer that pairs candidates and applies a caller
 *        supplied predicate to decide which pairs become merges.
 */

namespace hydra {

/// @brief A proposed merge folding the source node into the target node.
struct Merge {
  NodeId from;  ///< Node to be absorbed and removed.
  NodeId to;    ///< Node that survives and receives the merged attributes.
  /// @brief Returns a copy with both endpoints rewritten through a remapping.
  /// @param remapping Mapping from old node ids to their replacements.
  /// @return Merge with remapped endpoints.
  Merge remap(const std::map<NodeId, NodeId>& remapping) const;
};

/// @brief Streams a merge as "from -> to".
std::ostream& operator<<(std::ostream& out, const Merge& merge);

/// @brief Equality on both merge endpoints.
inline bool operator==(const Merge& lhs, const Merge& rhs) {
  return lhs.from == rhs.from && lhs.to == rhs.to;
}

/// @brief Ordered list of proposed merges.
using MergeList = std::list<Merge>;

/// @brief Interface that selects candidate nodes to test against a given node.
struct AssociationStrategy {
  virtual ~AssociationStrategy() = default;
  /// @brief Returns the candidate nodes to consider merging with the given node.
  /// @param layer Layer holding the candidate nodes.
  /// @param node Node whose merge candidates are requested.
  /// @return View over the candidate nodes.
  virtual spark_dsg::LayerView candidates(const SceneGraphLayer& layer,
                                          const SceneGraphNode& node) const = 0;
};

/// @brief Pairs nodes with their candidates and emits merges that pass a predicate.
struct MergeProposer {
  /// @brief Predicate deciding whether a candidate node may merge into another.
  using MergeCheck = std::function<bool(const SceneGraphNode&, const SceneGraphNode&)>;

  struct Config {
    config::VirtualConfig<AssociationStrategy> strategy;  ///< Strategy that selects merge candidates.
  } const config;

  /// @brief Constructs a proposer from its configuration.
  explicit MergeProposer(const Config& config) : config(config) {}

  /// @brief Finds merges among the given nodes and appends them to the output list.
  /// @param layer Layer the nodes belong to.
  /// @param view Nodes to consider as merge sources.
  /// @param should_merge Predicate gating each candidate pair.
  /// @param nodes_to_merge Output list extended with the accepted merges.
  void findMerges(const SceneGraphLayer& layer,
                  const spark_dsg::LayerView& view,
                  const MergeCheck& should_merge,
                  MergeList& nodes_to_merge) const;
};

/// @brief Declares the configuration fields for MergeProposer.
void declare_config(MergeProposer::Config& config);

}  // namespace hydra

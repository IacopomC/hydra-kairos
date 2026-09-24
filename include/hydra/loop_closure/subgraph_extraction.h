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
/**
 * @file subgraph_extraction.h
 * @brief Selects the neighborhood of nodes around a root node that feeds a
 *        descriptor. The neighborhood can be a fixed radius or one grown to
 *        meet a minimum node count between configured bounds.
 */
#pragma once
#include "hydra/common/dsg_types.h"

namespace hydra {

/// @brief Settings for choosing the nodes that make up a subgraph around a root.
struct SubgraphConfig {
  bool fixed_radius = true;  ///< Use max_radius_m directly when true; otherwise grow toward min_nodes.
  double max_radius_m;       ///< Largest neighborhood radius considered (m).
  double min_radius_m;       ///< Smallest neighborhood radius considered (m) when growing.
  size_t min_nodes;          ///< Target node count when the radius is not fixed.

  /// @brief Construct a fixed-radius configuration.
  /// @param radius_m Neighborhood radius in meters.
  SubgraphConfig(double radius_m);

  /// @brief Construct with default-initialized fields.
  SubgraphConfig();
};

/// @brief Collect the nodes within the configured neighborhood of a root node.
/// @param config Radius and node-count settings.
/// @param graph Scene graph to read positions and connectivity from.
/// @param root_node Node at the center of the neighborhood.
/// @param is_places True when extracting from the places layer, which uses places connectivity.
/// @return The set of node identifiers forming the subgraph.
std::set<NodeId> getSubgraphNodes(const SubgraphConfig& config,
                                  const DynamicSceneGraph& graph,
                                  NodeId root_node,
                                  bool is_places);

}  // namespace hydra

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
#include "hydra/common/dsg_types.h"

/**
 * @file minimum_spanning_tree.h
 * @brief Computes a minimum spanning tree over a scene graph layer, where edge weight is
 *        the Euclidean distance between the connected node positions. The result lists
 *        the chosen edges along with each node's degree and the set of leaf nodes.
 */

namespace hydra {

/// @brief A weighted edge between two nodes, ordered by distance for heap use.
struct MinimalEdge {
  NodeId source;    ///< Source node id.
  NodeId target;    ///< Target node id.
  double distance;  ///< Edge weight (Euclidean distance between endpoints).

  /// @brief Construct a default, unset edge.
  MinimalEdge() = default;

  /// @brief Construct an edge with explicit endpoints and weight.
  MinimalEdge(NodeId source, NodeId target, double distance)
      : source(source), target(target), distance(distance) {}

  /// @brief Order by distance so a min-heap can be built with std::greater.
  inline bool operator>(const MinimalEdge& other) const {
    return distance > other.distance;
  }
};

/// @brief Result of a minimum spanning tree computation.
struct MinimumSpanningTreeInfo {
  std::vector<MinimalEdge> edges;             ///< Edges selected for the spanning tree.
  std::unordered_set<NodeId> leaves;          ///< Nodes with degree one in the tree.
  std::unordered_map<NodeId, size_t> counts;  ///< Degree of each node within the tree.
};

/// @brief Predicate selecting which layer edges may participate in the tree.
using EdgeFilter = std::function<bool(const SceneGraphLayer&, const SceneGraphEdge&)>;

/// @brief Compute the minimum spanning tree over all edges of a layer.
/// @param layer Layer whose nodes and edges define the graph.
/// @return The selected edges, node degrees and leaf set.
MinimumSpanningTreeInfo getMinimumSpanningEdges(const SceneGraphLayer& layer);

/// @brief Compute the minimum spanning tree using only edges accepted by the filter.
/// @param layer Layer whose nodes and edges define the graph.
/// @param func Predicate that accepts or rejects each candidate edge.
/// @return The selected edges, node degrees and leaf set.
MinimumSpanningTreeInfo getMinimumSpanningEdges(const SceneGraphLayer& layer,
                                                const EdgeFilter& func);

}  // namespace hydra

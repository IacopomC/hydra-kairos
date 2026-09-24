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
 * @file graph_clustering.h
 * @brief Algorithms that partition a scene graph layer into clusters starting from
 *        a set of seed communities, used to group places into rooms. Provides a
 *        greedy modularity optimizer and a weight-ordered flood-fill clusterer,
 *        along with the result container that maps nodes to cluster labels.
 */

namespace hydra {

/// @brief A list of seed clusters, each a group of node IDs assigned to one community.
using InitialClusters = std::vector<std::vector<NodeId>>;
/// @brief Returns the weight to use for the edge between two nodes of a layer.
using EdgeWeightFunc = std::function<double(const SceneGraphLayer&, NodeId, NodeId)>;

/// @brief Outcome of clustering a layer: the resulting communities and a per-node lookup.
struct ClusterResults {
  using Cluster = std::unordered_set<NodeId>;  ///< Set of node IDs belonging to one cluster.
  using Clusters = std::map<size_t, Cluster>;  ///< Clusters indexed by integer label.

  Clusters clusters;              ///< Members of each cluster, keyed by label.
  std::map<NodeId, size_t> labels;  ///< Cluster label assigned to each node.
  size_t total_iters;             ///< Number of refinement iterations that ran.
  bool valid = false;             ///< True once the results have been populated.

  /// @brief Reset to an empty, invalid state.
  void clear();

  /// @brief Populate clusters and labels directly from seed clusters, one label per seed.
  /// @param initial_clusters Seed clusters to copy in verbatim.
  void fillFromInitialClusters(const InitialClusters& initial_clusters);
};

/// @brief Assign nodes to communities by greedily maximizing modularity, using stored
/// edge weights. Seed clusters stay fixed; remaining nodes are pulled into the community
/// that yields the largest modularity gain.
/// @param layer Graph layer to cluster.
/// @param initial_clusters Seed communities that anchor the labeling.
/// @param max_iters Maximum number of passes over the unlabeled nodes.
/// @param gamma Resolution parameter weighting the degree-based penalty term.
/// @return Cluster assignments for the layer.
ClusterResults clusterGraphByModularity(const SceneGraphLayer& layer,
                                        const InitialClusters& initial_clusters,
                                        size_t max_iters = 5,
                                        double gamma = 1.0);

/// @brief Modularity clustering variant that derives edge weights from a caller-supplied
/// function rather than the stored edge weight.
/// @param layer Graph layer to cluster.
/// @param initial_clusters Seed communities that anchor the labeling.
/// @param edge_weight_func Supplies the weight for each edge; negative values are clamped to zero.
/// @param max_iters Maximum number of passes over the unlabeled nodes.
/// @param gamma Resolution parameter weighting the degree-based penalty term.
/// @return Cluster assignments for the layer.
ClusterResults clusterGraphByModularity(const SceneGraphLayer& layer,
                                        const InitialClusters& initial_clusters,
                                        const EdgeWeightFunc& edge_weight_func,
                                        size_t max_iters = 5,
                                        double gamma = 1.0);

/// @brief Grow the seed clusters outward by a weight-ordered flood fill, assigning each
/// unlabeled node to the cluster reached first across the highest-weight boundary edges.
/// @param layer Graph layer to cluster.
/// @param initial_clusters Seed communities to grow from.
/// @return Cluster assignments for the layer.
ClusterResults clusterGraphByNeighbors(const SceneGraphLayer& layer,
                                       const InitialClusters& initial_clusters);

}  // namespace hydra

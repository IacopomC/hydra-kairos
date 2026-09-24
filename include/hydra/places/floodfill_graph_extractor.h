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

#include <queue>

#include "hydra/common/dsg_types.h"
#include "hydra/places/graph_extractor_interface.h"
#include "hydra/places/gvd_voxel.h"
#include "hydra/places/voxel_templates.h"
#include "hydra/reconstruction/voxel_types.h"

/**
 * @file floodfill_graph_extractor.h
 * @brief Place-graph extractor that floodfills along the Voronoi voxels: corner
 *        voxels become nodes, the voxels between them become edges, and a
 *        subsequent pass splits edges that stray too far from their supporting
 *        voxels. Tracks per-voxel and per-edge membership so the graph can be
 *        maintained incrementally.
 */

namespace hydra::places {

/// @brief Floodfill state attached to a single voxel: which node or edge it belongs to.
struct VoxelGraphInfo {
  VoxelGraphInfo();

  /// @brief Construct as a node voxel with the given id.
  VoxelGraphInfo(NodeId id, bool is_from_split);

  NodeId id;             ///< Owning node id (when a node) or source node of the edge.
  bool is_node;          ///< Whether the voxel is a graph node rather than an edge interior.
  bool is_split_node;    ///< Whether the node was introduced by an edge split.
  size_t edge_id;        ///< Owning edge id when the voxel is part of an edge.
};

/// @brief State of an edge being floodfilled: its supporting voxels and connections.
struct EdgeInfo {
  EdgeInfo() = default;

  /// @brief Construct an edge with the given id originating at a source node.
  EdgeInfo(size_t id, NodeId source);

  size_t id;                              ///< Edge id.
  NodeId source;                          ///< Node the edge grew from.
  GlobalIndexSet indices;                 ///< Voxel indices making up the edge.
  std::set<NodeId> node_connections;      ///< Nodes this edge reaches.
  std::set<size_t> connections;           ///< Other edges this edge meets.
};

/// @brief Candidate point for splitting an edge, ordered by distance from the edge.
struct EdgeSplitSeed {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// @brief Construct a split seed at a voxel a given distance from its edge.
  EdgeSplitSeed(const GlobalIndex& index, double distance_to_edge, size_t edge_id);

  GlobalIndex index;          ///< Voxel proposed as a split point.
  double distance_to_edge;    ///< How far the voxel lies from the straight edge.
  size_t edge_id;             ///< Edge the seed would split.
};

/// @brief Order split seeds so the furthest-from-edge seed has highest priority.
bool operator<(const EdgeSplitSeed& lhs, const EdgeSplitSeed& rhs);

/// @brief A provisional edge described by its node sequence and supporting voxels.
struct PseudoEdgeInfo {
  std::vector<NodeId> nodes;   ///< Ordered nodes along the edge.
  GlobalIndices indices;       ///< Voxel indices supporting the edge.
};

/// @brief Stream a human-readable view of a voxel's floodfill state.
std::ostream& operator<<(std::ostream& out, const VoxelGraphInfo& info);

/// @brief Stream a human-readable view of an edge's floodfill state.
std::ostream& operator<<(std::ostream& out, const EdgeInfo& info);

/// @brief Extracts a place graph by floodfilling along the GVD's Voronoi voxels.
class FloodfillGraphExtractor : public GraphExtractorInterface {
 public:
  using Ptr = std::unique_ptr<FloodfillGraphExtractor>;  ///< Unique-pointer alias.

  using NodeChildMap = std::unordered_map<NodeId, GlobalIndexSet>;   ///< Voxels owned by each node.
  using IndexGraphInfoMap = GlobalIndexMap<VoxelGraphInfo>;          ///< Floodfill state per voxel index.
  using EdgeInfoMap = std::unordered_map<size_t, EdgeInfo>;          ///< Edge records keyed by edge id.
  using EdgeSplitQueue = std::priority_queue<EdgeSplitSeed, std::vector<EdgeSplitSeed>>;  ///< Pending edge splits, worst deviation first.

  /// @brief Construct from the floodfill extractor configuration.
  explicit FloodfillGraphExtractor(const FloodfillExtractorConfig& config);

  virtual ~FloodfillGraphExtractor();

  void clearGvdIndex(const GlobalIndex& index) override;

  void removeDistantIndex(const GlobalIndex& index) override;

  void extract(const GvdLayer& layer, uint64_t timestamp_ns) override;

 protected:
  /// @brief Create a place node for a voxel and seed the floodfill frontier.
  void addNewPlaceNode(const GvdLayer& layer,
                       const GvdVoxel& voxel,
                       const GlobalIndex& index,
                       bool is_from_split = false);

  /// @brief Drop all bookkeeping for a node.
  void clearNodeInfo(NodeId node_id);

  /// @brief Drop all bookkeeping for an edge, optionally releasing its voxels.
  void clearEdgeInfo(size_t edge_id, bool clear_indices = true);

  /// @brief Remove a node's voxels from the per-voxel state map.
  void removeNodeIndex(NodeId node_id);

  /// @brief Remove an edge's voxels from the per-voxel state map.
  void removeEdgeIndices(size_t edge_id, bool clear_indices);

  /// @brief Push a neighbor of a voxel onto the floodfill frontier.
  void addNeighborToFrontier(const VoxelGraphInfo& info,
                             const GlobalIndex& neighbor_index);

  /// @brief Record an adjacency between two voxels' nodes/edges.
  /// @return Whether a new connection was established.
  bool updateEdgeMaps(const VoxelGraphInfo& info, const VoxelGraphInfo& neighbor_info);

  /// @brief Add an edge to the place graph between two floodfill-adjacent voxels.
  void addEdgeToGraph(const GvdLayer& layer,
                      const VoxelGraphInfo& curr_info,
                      const VoxelGraphInfo& neighbor_info);

  /// @brief Identify edge voxels that deviate too far and queue them for splitting.
  void findBadEdgeIndices(const EdgeInfo& info);

  /// @brief Promote corner voxels discovered during floodfill into new nodes.
  void findNewVertices(const GvdLayer& layer);

  /// @brief Try to merge two nearby nodes during floodfill.
  /// @return Whether a merge was performed.
  bool attemptNodeMerge(const GvdLayer& layer,
                        const VoxelGraphInfo& curr_info,
                        const VoxelGraphInfo& neighbor_info);

  /// @brief Floodfill from current nodes to grow edges, optionally merging nodes.
  void extractEdges(const GvdLayer& layer, bool allow_merging = false);

  /// @brief Drop edge connections whose voxels have been removed.
  void filterRemovedConnections();

  /// @brief Split edges that deviate too far from their supporting voxels.
  void splitEdges(const GvdLayer& layer);

  /// @brief Pop the next voxel index from the floodfill frontier.
  GlobalIndex popFromFloodfillFrontier();

  /// @brief Reset the per-pass connection tracking, optionally the modified-voxel queue.
  void clearNewConnections(bool clear_modified_voxels);

  /// @brief Decide whether a voxel qualifies as a graph vertex.
  bool isVertex(const GvdLayer& layer, const GvdVoxel& voxel, const GlobalIndex& index);

 protected:
  FloodfillExtractorConfig config_;     ///< Floodfill extractor configuration.
  CornerFinder corner_finder_;          ///< Template matcher detecting corner (vertex) voxels.

  std::queue<GlobalIndex> floodfill_frontier_;  ///< Voxels awaiting floodfill expansion.

  IndexGraphInfoMap index_graph_info_map_;  ///< Per-voxel node/edge membership state.
  NodeChildMap node_child_map_;             ///< Voxels owned by each node.

  size_t next_edge_id_;                                  ///< Next edge id to allocate.
  EdgeInfoMap edge_info_map_;                            ///< Edge records keyed by edge id.
  std::map<NodeId, std::set<size_t>> node_edge_id_map_;       ///< Edges owned by each node.
  std::map<NodeId, std::set<size_t>> node_edge_connections_;  ///< Edges incident on each node.

  EdgeSplitQueue edge_split_queue_;  ///< Pending edge-split candidates.

  std::map<size_t, std::set<size_t>> checked_edges_;  ///< Edge pairs already compared for connectivity.
  std::set<size_t> connected_edges_;                  ///< Edges already linked this pass.

  inline static const auto registration_ =
      config::RegistrationWithConfig<GraphExtractorInterface,
                                     FloodfillGraphExtractor,
                                     FloodfillExtractorConfig>(
          "FloodfillGraphExtractor");  ///< Factory registration under "FloodfillGraphExtractor".
};

}  // namespace hydra::places

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
#include <queue>

#include "hydra/common/dsg_types.h"
#include "hydra/places/graph_extractor_config.h"
#include "hydra/places/gvd_graph.h"
#include "hydra/places/gvd_voxel.h"
#include "hydra/reconstruction/voxel_types.h"

/**
 * @file graph_extractor_interface.h
 * @brief Base class for converting the GVD into a place graph. Maintains the
 *        output scene-graph layer, the intermediate GVD graph, the voxel-to-node
 *        mapping, and the queues of modified and deleted elements that concrete
 *        extractors update each step.
 */

namespace hydra::places {

struct GvdParentTracker;  ///< Forward declaration; supplies surface-parent information.

/// @brief Common interface and shared state for GVD-to-place-graph extractors.
class GraphExtractorInterface {
 public:
  using Ptr = std::shared_ptr<GraphExtractorInterface>;             ///< Shared-pointer alias.
  using NodeIndexMap = std::unordered_map<NodeId, GlobalIndex>;     ///< Map from place node to its source voxel index.

  /// @brief Construct with the shared extractor configuration.
  explicit GraphExtractorInterface(const GraphExtractorConfig& config);

  virtual ~GraphExtractorInterface();

  /// @brief Queue a GVD voxel index as modified for the next extraction.
  virtual void pushGvdIndex(const GlobalIndex& index);

  /// @brief Remove all extractor state associated with a GVD voxel.
  virtual void clearGvdIndex(const GlobalIndex& index) = 0;

  /// @brief Drop a voxel that has moved out of the active region.
  virtual void removeDistantIndex(const GlobalIndex& index) = 0;

  /// @brief Update the place graph from the current GVD state.
  virtual void extract(const GvdLayer& layer, uint64_t timestamp_ns) = 0;

  /// @brief Populate each place node's basis-point/parent information from the tracker.
  void fillParentInfo(const GvdLayer& gvd, const GvdParentTracker& parents);

  /// @brief Access the extracted place-graph layer.
  const SceneGraphLayer& getGraph() const;

  /// @brief Access the intermediate GVD graph.
  const GvdGraph& getGvdGraph() const;

  /// @brief Return the ids of nodes currently in the graph.
  std::unordered_set<NodeId> getActiveNodes() const;

  /// @brief Return the ids of nodes removed since the last clearDeleted().
  const std::unordered_set<NodeId>& getDeletedNodes() const;

  /// @brief Return the endpoints of edges removed since the last clearDeleted().
  const std::vector<NodeId>& getDeletedEdges() const;

  /// @brief Clear the pending deleted-node and deleted-edge records.
  void clearDeleted();

  /// @brief Access the node-to-voxel-index map.
  const NodeIndexMap& getIndexMap() const { return node_index_map_; }

 protected:
  /// @brief Add a place node for a voxel and return its assigned id.
  NodeId addPlaceToGraph(const GvdLayer& layer,
                         const GvdVoxel& voxel,
                         const GlobalIndex& index);

  /// @brief Build edge attributes (length and clearance) between two place nodes.
  EdgeAttributes::Ptr makeEdgeInfo(const GvdLayer& layer,
                                   NodeId source_id,
                                   NodeId target_id) const;

  /// @brief Pop the next modified voxel index from the queue.
  GlobalIndex popFromModifiedQueue();

  /// @brief Remove a node from the place graph and record it as deleted.
  void removeGraphNode(NodeId node);

  /// @brief Insert or update an edge between two nodes.
  /// @param is_heursitic Whether the edge is a heuristic (non-GVD-derived) edge.
  void updateGraphEdge(NodeId source,
                       NodeId target,
                       EdgeAttributes::Ptr&& attrs,
                       bool is_heursitic = false);

  /// @brief Add a node to the place graph with optional attributes.
  void addGraphNode(NodeId node, PlaceNodeAttributes::Ptr&& attrs = nullptr);

  /// @brief Remove an edge between two nodes and record it as deleted.
  void removeGraphEdge(NodeId source, NodeId target);

  /// @brief Merge one node into another, transferring its edges.
  void mergeGraphNodes(NodeId from, NodeId to);

  /// @brief Recompute heuristic edges over the current graph.
  void updateHeuristicEdges(const GvdLayer& layer);

 protected:
  const GraphExtractorConfig config_;                ///< Shared extractor configuration.
  NodeSymbol next_node_id_;                          ///< Symbol used to allocate the next place node id.
  GvdGraph::Ptr gvd_;                                ///< Intermediate GVD graph.

  NodeIndexMap node_index_map_;                      ///< Map from place node id to source voxel index.
  std::queue<GlobalIndex> modified_voxel_queue_;     ///< Voxel indices awaiting extraction.

  std::map<EdgeKey, bool> heuristic_edges_;          ///< Heuristic edges keyed by endpoint pair.
  std::unordered_set<NodeId> deleted_nodes_;         ///< Nodes removed since the last clearDeleted().
  std::vector<NodeId> deleted_edges_;                ///< Endpoints of edges removed since the last clearDeleted().

 private:
  IsolatedSceneGraphLayer::Ptr graph_;               ///< Backing place-graph layer.
};

}  // namespace hydra::places

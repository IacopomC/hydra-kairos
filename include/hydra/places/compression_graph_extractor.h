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

#include "hydra/places/graph_extractor_interface.h"
#include "hydra/places/gvd_merge_policies.h"

/**
 * @file compression_graph_extractor.h
 * @brief Place-graph extractor that compresses the GVD by clustering nearby
 *        Voronoi members onto a coarse spatial grid. Each grid cell yields one
 *        compressed node whose edges are accumulated from the underlying GVD
 *        member adjacencies, with bookkeeping to add, merge, and archive nodes
 *        as the diagram changes.
 */

namespace hydra::places {

/// @brief A GVD voxel index paired with a pointer to its voxel.
struct IndexVoxelPair {
  GlobalIndex index;        ///< Voxel index.
  const GvdVoxel* voxel;    ///< Pointer to the voxel data.
};

/// @brief A coarse-grid node aggregating one or more GVD members and their edges.
struct CompressedNode {
  using CompressedNodeMap = std::unordered_map<uint64_t, CompressedNode>;  ///< Map of compressed-node id to node.
  uint64_t node_id;                          ///< Id of this compressed node.
  std::set<uint64_t> siblings;               ///< Ids of adjacent compressed nodes.
  std::set<uint64_t> active_refs;            ///< Ids of GVD members currently mapped here.
  std::set<uint64_t> archived_refs;          ///< Ids of GVD members that have been archived.
  std::unordered_map<uint64_t, std::map<uint64_t, uint64_t>> sibling_support;  ///< Per-sibling map of supporting GVD edge endpoints.
  std::map<uint64_t, uint64_t> sibling_ref_counts;  ///< Number of supporting observations per sibling.
  uint64_t best_gvd_id;                      ///< Id of the GVD member chosen as this node's representative.
  bool in_graph = false;                     ///< Whether this node currently exists in the place graph.

  /// @brief Construct an empty compressed node with the given id.
  explicit CompressedNode(uint64_t node_id);

  /// @brief Record that a GVD edge supports the connection to a sibling node.
  void addEdgeObservation(uint64_t gvd_id,
                          uint64_t neighbor_gvd_id,
                          uint64_t sibling_id);

  /// @brief Remove a single supporting GVD edge observation.
  /// @return Whether the sibling connection lost all support and should be dropped.
  bool removeEdgeObservation(uint64_t gvd_id, uint64_t neighbor_gvd_id);

  /// @brief Remove all observations contributed by a GVD member.
  /// @return Sibling ids whose connection to this node lost all support.
  std::list<uint64_t> removeEdgeObservations(uint64_t gvd_id, CompressedNodeMap& nodes);

  /// @brief Re-key observations from one GVD member id to another.
  void mergeObservations(uint64_t original_id, uint64_t new_id);

  /// @brief Absorb another compressed node's members and edge support into this one.
  void merge(CompressedNode& other, CompressedNodeMap& nodes);
};

using IndexVoxelQueue = std::list<IndexVoxelPair>;  ///< Ordered list of index/voxel pairs to process.

/// @brief Extracts a place graph by compressing the GVD onto a coarse spatial grid.
class CompressionGraphExtractor : public GraphExtractorInterface {
 public:
  using Ptr = std::unique_ptr<CompressionGraphExtractor>;  ///< Unique-pointer alias.

  /// @brief Construct from the compression extractor configuration.
  explicit CompressionGraphExtractor(const CompressionExtractorConfig& config);

  virtual ~CompressionGraphExtractor();

  void clearGvdIndex(const GlobalIndex& index) override;

  void removeDistantIndex(const GlobalIndex& index) override;

  void extract(const GvdLayer& layer, uint64_t timestamp_ns) override;

  /// @brief Access the current compressed-node records keyed by id.
  inline const std::unordered_map<uint64_t, CompressedNode>& getCompressedNodeInfo()
      const {
    return compressed_info_map_;
  }

  /// @brief Access the mapping from merged compressed-node ids to their survivors.
  inline const std::unordered_map<uint64_t, uint64_t>& getCompressedRemapping() const {
    return compressed_remapping_;
  }

 protected:
  /// @brief Collect the queued modified voxels that are still valid GVD members.
  void fillSeenVoxels(const GvdLayer& layer,
                      uint64_t timestamp_ns,
                      IndexVoxelQueue& seen_voxels);

  /// @brief Refresh GVD member records for the seen voxels.
  void updateNodeInfoMap(const GvdLayer& layer,
                         const IndexVoxelQueue& update_info,
                         uint64_t timestamp_ns);

  /// @brief Update the intermediate GVD graph adjacency from the seen voxels.
  void updateGvdGraph(const IndexVoxelQueue& update_info, uint64_t timestamp_ns);

  /// @brief Remove GVD members that are no longer valid in the layer.
  void pruneInvalidGvdMembers(const GvdLayer& layer);

  /// @brief Detach a compressed node, optionally deleting it from the graph.
  void clearCompressionId(uint64_t node_id, bool is_delete);

  /// @brief Map a world point to its coarse compression-grid index.
  GlobalIndex getCompressedIndex(const Eigen::Vector3d& point) const;

  /// @brief Pop a recycled compressed-node id, or allocate a fresh one.
  uint64_t getNextId();

  /// @brief Look up the place-graph node id backing a GVD member.
  NodeId getPlaceId(uint64_t gvd_id) const;

  /// @brief Assign a GVD member to its compression cell, creating a node if needed.
  void compressNode(uint64_t node_id, const GvdMemberInfo& node);

  /// @brief Push each compressed node's chosen representative into the place graph.
  void assignCompressedNodeAttributes();

  /// @brief Merge compressed nodes that fall within the merge distance.
  void mergeNearbyNodes();

  /// @brief Rebuild place-graph edges from the compressed nodes' sibling support.
  void updateCompressedEdges(const GvdLayer& layer);

  /// @brief Remove the GVD members owned by a compressed node being deleted.
  void deleteChildren(const CompressedNode& info);

  /// @brief Finalize archived nodes, removing them from the graph.
  void clearArchived();

  /// @brief Add or refresh a GVD member at the given index and position.
  void updateNode(const GlobalIndex& index,
                  const Eigen::Vector3d& position,
                  double distance,
                  uint8_t num_basis_points);

  /// @brief Run consistency checks on the compressed graph against the layer.
  void validate(const GvdLayer& layer) const;

  /// @brief Find an existing cluster a node should join, if any.
  std::optional<uint64_t> findCluster(uint64_t node_id,
                                      const GvdMemberInfo& node,
                                      const std::set<uint64_t>& clusters);

  /// @brief Merge a neighboring compressed node into the current one.
  void mergeCompressedNodes(uint64_t curr_node_id,
                            CompressedNode& curr_node,
                            uint64_t neighbor_node_id);

 protected:
  CompressionExtractorConfig config_;            ///< Compression extractor configuration.
  double compression_factor_;                    ///< Reciprocal of compression_distance_m; scales points onto the coarse grid.
  GlobalIndexMap<uint64_t> index_id_map_;        ///< Map from GVD voxel index to GVD member id.
  std::unique_ptr<MergePolicy> merge_policy_;    ///< Policy choosing the representative on a merge.

  uint64_t next_id_;                             ///< Next never-used compressed-node id.
  std::list<uint64_t> id_queue_;                 ///< Freed compressed-node ids awaiting reuse.

  std::unordered_set<uint64_t> updated_nodes_;   ///< Compressed nodes touched this update.
  std::unordered_set<uint64_t> to_archive_;      ///< Compressed nodes pending archival.

  std::unordered_map<uint64_t, CompressedNode> compressed_info_map_;  ///< Compressed-node records keyed by id.
  GlobalIndexMap<std::set<uint64_t>> compressed_index_map_;           ///< Compressed-node ids occupying each grid cell.
  std::unordered_map<uint64_t, GlobalIndex> compressed_id_map_;       ///< Grid cell of each compressed node.
  std::unordered_map<uint64_t, uint64_t> compressed_remapping_;       ///< Map from merged ids to surviving ids.

  std::unordered_set<NodeId> archived_node_ids_;  ///< Place-graph ids that have been archived.

  inline static const auto registration_ =
      config::RegistrationWithConfig<GraphExtractorInterface,
                                     CompressionGraphExtractor,
                                     CompressionExtractorConfig>(
          "CompressionGraphExtractor");  ///< Factory registration under "CompressionGraphExtractor".
};

}  // namespace hydra::places

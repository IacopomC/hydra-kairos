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
#include <memory>

#include "hydra/common/dsg_types.h"
#include "hydra/common/output_sink.h"

namespace kimera_pgmo {
class MeshDelta;
}  // namespace kimera_pgmo

/**
 * @file mesh_segmenter.h
 * @brief Clusters labeled mesh vertices into object nodes and maintains those
 *        object nodes in the scene graph as the mesh grows.
 */

namespace hydra {

/// @brief One spatial cluster of mesh vertices: its centroid and vertex indices.
struct Cluster {
  Eigen::Vector3d centroid;      ///< Mean position of the clustered vertices.
  std::vector<size_t> indices;   ///< Mesh vertex indices belonging to the cluster.
};

/// @brief Mesh vertex indices grouped by semantic label.
using LabelIndices = std::map<uint32_t, std::vector<size_t>>;

/// @brief Segments the active mesh into per-label object clusters and writes
/// them to the scene graph, adding new object nodes, updating existing ones,
/// and merging overlapping detections.
class MeshSegmenter {
 public:
  using Clusters = std::vector<Cluster>;                ///< Clusters for a single label.
  using LabelClusters = std::map<uint32_t, Clusters>;   ///< Clusters grouped by label.
  using Sink = OutputSink<uint64_t,
                          const kimera_pgmo::MeshDelta&,
                          const std::vector<size_t>&,
                          const LabelIndices&>;          ///< Observer hook for detection results.

  /// @brief Settings controlling clustering and the object nodes produced.
  struct Config {
    char prefix = 'O';                          ///< Node id prefix for object nodes.
    LayerId layer_id = DsgLayers::OBJECTS;       ///< Scene-graph layer that receives objects.
    double cluster_tolerance = 0.25;             ///< Max neighbor spacing within a cluster (m).
    size_t min_cluster_size = 40;                ///< Smallest accepted cluster size (vertices).
    size_t max_cluster_size = 100000;            ///< Largest accepted cluster size (vertices).
    BoundingBox::Type bounding_box_type = BoundingBox::Type::AABB;  ///< Bounding-box style for object attributes.
    std::string timer_namespace = "frontend/objects";  ///< Profiling namespace for timers.
    std::vector<Sink::Factory> sinks;           ///< Factories for detection-result observers.
  } const config;

  /// @brief Construct the segmenter for a fixed set of object labels.
  /// @param config Segmenter settings.
  /// @param labels Semantic labels to treat as objects.
  explicit MeshSegmenter(const Config& config, const std::set<uint32_t>& labels);

  /// @brief Cluster the active mesh vertices into objects per label.
  /// @param timestamp_ns Time of the current step in nanoseconds.
  /// @param active Mesh delta whose active vertices are clustered.
  /// @return Clusters grouped by semantic label.
  LabelClusters detect(uint64_t timestamp_ns,
                       const kimera_pgmo::MeshDelta& active);

  /// @brief Add, update, and merge object nodes in the graph from clusters.
  /// @param timestamp Time of the current step in nanoseconds.
  /// @param active Mesh delta the clusters were computed from.
  /// @param clusters Per-label clusters produced by detect().
  /// @param graph Scene graph updated in place.
  void updateGraph(uint64_t timestamp,
                   const kimera_pgmo::MeshDelta& active,
                   const LabelClusters& clusters,
                   DynamicSceneGraph& graph);

  /// @brief Return the set of object nodes currently considered active.
  std::unordered_set<NodeId> getActiveNodes() const;

 private:
  /// @brief Refresh attributes of previously added nodes against the new mesh.
  void updateOldNodes(const kimera_pgmo::MeshDelta& active, DynamicSceneGraph& graph);

  /// @brief Create a new object node in the graph from a cluster.
  void addNodeToGraph(DynamicSceneGraph& graph,
                      const Cluster& cluster,
                      uint32_t label,
                      uint64_t timestamp);

  /// @brief Update an existing object node from a cluster.
  void updateNodeInGraph(DynamicSceneGraph& graph,
                         const Cluster& cluster,
                         const SceneGraphNode& node,
                         uint64_t timestamp);

  /// @brief Merge active nodes of one label whose detections overlap.
  void mergeActiveNodes(DynamicSceneGraph& graph, uint32_t label);

 private:
  NodeSymbol next_node_id_;                            ///< Next object node id to assign.
  std::set<uint32_t> labels_;                          ///< Labels treated as objects.
  std::map<uint32_t, std::set<NodeId>> active_nodes_;  ///< Active object nodes per label.
  Sink::List sinks_;                                   ///< Detection-result observers.
};

/// @brief Declare the config fields for the mesh segmenter.
void declare_config(MeshSegmenter::Config& config);

}  // namespace hydra

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
#include <kimera_pgmo/mesh_delta.h>
#include <pcl/common/centroid.h>

#include <memory>

#include "hydra/common/dsg_types.h"
#include "hydra/frontend/place_2d_split_logic.h"
#include "hydra/frontend/surface_places_interface.h"

/**
 * @file place_2d_segmenter.h
 * @brief Surface place extractor that clusters labeled mesh vertices into 2D
 *        places, splits oversized clusters, and maintains the places and their
 *        connectivity in the scene graph.
 */

namespace hydra {

/// @brief Extracts 2D surface places from the mesh. Vertices are clustered per
/// label, large clusters are split into ellipse-bounded places, and the
/// resulting nodes plus their overlap edges are written to the scene graph.
/// Places are tracked as active or semi-active as the mesh is archived.
class Place2dSegmenter : public SurfacePlacesInterface {
 public:
  using IndicesVector = pcl::IndicesPtr::element_type;     ///< Plain index container type.
  using LabelIndices = std::map<uint32_t, pcl::IndicesPtr>; ///< Vertex indices grouped by label.
  using MeshVertexCloud = Place2d::CloudT;                 ///< Mesh vertex position container.
  using Places = std::vector<Place2d>;                     ///< Places for a single label.
  using LabelPlaces = std::map<uint32_t, Places>;          ///< Places grouped by label.

  /// @brief Settings controlling clustering, splitting, and connectivity.
  struct Config {
    char prefix = 'Q';                       ///< Node id prefix for place nodes.
    double cluster_tolerance = 1;            ///< Max neighbor spacing within a cluster (m).
    size_t min_cluster_size = 600;           ///< Smallest accepted cluster size (vertices).
    size_t max_cluster_size = 100000;        ///< Largest accepted cluster size (vertices).
    double pure_final_place_size = 3;        ///< Target extent below which a place stops splitting (m).
    size_t min_final_place_points = 1000;    ///< Minimum points a place keeps when splitting.
    double place_overlap_threshold = 0.1;    ///< Minimum ellipse overlap to connect two places.
    double place_max_neighbor_z_diff = 0.5;  ///< Maximum height difference between connected places (m).
    double connection_ellipse_scale_factor = 1;  ///< Scale applied to fitted bounding ellipses.
    std::set<uint32_t> labels;               ///< Semantic labels treated as surfaces.
  } const config;

  /// @brief Construct the segmenter from its configuration.
  explicit Place2dSegmenter(const Config& config);

  /// @brief Cluster the latest mesh into 2D places per label.
  void detect(const ActiveWindowOutput& msg,
              const kimera_pgmo::MeshDelta& mesh_delta,
              const DynamicSceneGraph& graph) override;

  /// @brief Return the set of place nodes currently considered active.
  NodeIdSet getActiveNodes() const override;

  /// @brief Write detected places into the graph and update their edges.
  void updateGraph(uint64_t timestamp_ns,
                   const ActiveWindowOutput&,
                   DynamicSceneGraph& graph) override;

 private:
  /// @brief Decide whether two places should be connected and compute the edge.
  bool frontendAddPlaceConnection(const Place2dNodeAttributes& attrs1,
                                  const Place2dNodeAttributes& attrs2,
                                  EdgeAttributes& edge_weight);

  /// @brief Cluster and split mesh vertices for one label into 2D places.
  Places findPlaces(const Mesh::Positions& positions,
                    const kimera_pgmo::MeshDelta& delta,
                    const pcl::IndicesPtr& indices,
                    double connection_ellipse_scale_factor) const;

  /// @brief Archive places whose vertices have all left the active mesh.
  /// @return The nodes that were archived.
  std::set<NodeId> archiveOldObjects(const DynamicSceneGraph& graph,
                                     uint64_t latest_timestamp);

  /// @brief Group vertex indices by their semantic label.
  LabelIndices getLabelIndices(const Mesh::Labels& labels,
                               const IndicesVector& indices) const;

  /// @brief Create a new place node in the graph.
  /// @return The id assigned to the new node.
  NodeSymbol addPlaceToGraph(DynamicSceneGraph& graph,
                             const Place2d& place,
                             uint32_t label,
                             uint64_t timestamp);

  /// @brief Update an existing place node from a detected place.
  void updatePlaceInGraph(const Place2d& place,
                          const SceneGraphNode& node,
                          uint64_t timestamp);

 private:
  LabelPlaces detected_label_places_;  ///< Places detected in the latest detect() call, per label.

  NodeSymbol next_node_id_;            ///< Next place node id to assign.

  std::list<NodeId> nodes_to_remove_;  ///< Nodes scheduled for removal on the next graph update.

  size_t num_archived_vertices_;       ///< Count of mesh vertices already archived out of the window.
  std::map<uint32_t, std::set<NodeId>> active_places_;      ///< Active place nodes per label.
  std::map<uint32_t, std::set<NodeId>> semiactive_places_;  ///< Places partly archived but still updated.
  std::map<NodeId, uint64_t> active_place_timestamps_;      ///< Last update time per active place.

  inline static const auto registration_ =
      config::RegistrationWithConfig<SurfacePlacesInterface, Place2dSegmenter, Config>(
          "place_2d");  ///< Self-registration under the name "place_2d".
};

/// @brief Declare the config fields for the 2D place segmenter.
void declare_config(Place2dSegmenter::Config& config);

}  // namespace hydra

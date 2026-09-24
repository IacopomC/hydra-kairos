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
#include <unordered_set>
#include <vector>

#include "hydra/common/dsg_types.h"

/**
 * @file nearest_neighbor_utilities.h
 * @brief KD-tree backed nearest-neighbor search over scene graph nodes and over raw 3D
 *        points. Node finders are built from a fixed set of nodes and can be grouped by
 *        semantic label; the point search returns the single closest stored point.
 */

namespace hydra {

/// @brief KD-tree over a fixed set of layer node positions for proximity queries.
class NearestNodeFinder {
 public:
  /// @brief Receives a found node as (node id, index into the finder's node set, squared distance).
  using Callback = std::function<void(NodeId, size_t, double)>;
  /// @brief Predicate selecting which layer nodes to include.
  using Filter = std::function<bool(const SceneGraphNode&)>;
  using Ptr = std::unique_ptr<NearestNodeFinder>;  ///< Owning pointer alias.

  /// @brief Build a finder over the given nodes of a layer.
  NearestNodeFinder(const SceneGraphLayer& layer, const std::vector<NodeId>& nodes);

  /// @brief Build a finder over the given set of nodes of a layer.
  NearestNodeFinder(const SceneGraphLayer& layer,
                    const std::unordered_set<NodeId>& nodes);

  virtual ~NearestNodeFinder();

  /// @brief Build a finder over the layer nodes accepted by the filter.
  /// @param layer Layer to draw nodes from.
  /// @param filter Predicate selecting nodes to include.
  /// @return A finder, or nullptr if no node passes the filter.
  static Ptr fromLayer(const SceneGraphLayer& layer, const Filter& filter);

  /// @brief Invoke the callback for the nearest nodes to a position.
  /// @param position Query point.
  /// @param num_to_find Number of neighbors to report.
  /// @param skip_first If true, drop the closest result (useful when the query is itself a node).
  /// @param callback Invoked once per found neighbor.
  void find(const Eigen::Vector3d& position,
            size_t num_to_find,
            bool skip_first,
            const Callback& callback);

  /// @brief Invoke the callback for every node within a radius of a position.
  /// @param position Query point.
  /// @param radius_m Search radius in meters.
  /// @param skip_first If true, drop the closest result.
  /// @param callback Invoked once per found neighbor.
  /// @return Number of neighbors reported.
  size_t findRadius(const Eigen::Vector3d& position,
                    double radius_m,
                    bool skip_first,
                    const Callback& callback);

  const size_t num_nodes;  ///< Number of nodes indexed by this finder.

 private:
  struct Detail;
  std::unique_ptr<Detail> internals_;  ///< Hidden KD-tree implementation.
};

/// @brief Map from semantic label to a node finder built over nodes of that label.
using SemanticNodeFinders =
    std::map<SemanticNodeAttributes::Label, std::unique_ptr<NearestNodeFinder>>;

/// @brief Build one node finder per semantic label present in the layer.
/// @param layer Layer to draw nodes from.
/// @param finders Output map, cleared and repopulated.
/// @param use_active If false, active nodes are excluded from the finders.
/// @return Total number of nodes distributed across the finders.
size_t makeSemanticNodeFinders(const SceneGraphLayer& layer,
                               SemanticNodeFinders& finders,
                               bool use_active = false);

/// @brief KD-tree nearest-neighbor search over a fixed set of 3D points.
class PointNeighborSearch {
 public:
  /// @brief Build the search tree over the given points.
  explicit PointNeighborSearch(const std::vector<Eigen::Vector3f>& points);
  virtual ~PointNeighborSearch();

  /// @brief Find the nearest stored point to the query.
  /// @param query_point The query point.
  /// @param distance_squared Output squared distance to the nearest neighbor.
  /// @param index Output index of the nearest neighbor in the stored points.
  /// @return True if a nearest neighbor was found, false otherwise.
  bool search(const Eigen::Vector3f& query_point,
              float& distance_squared,
              size_t& index) const;

 private:
  struct Detail;
  std::unique_ptr<Detail> internals_;  ///< Hidden KD-tree implementation.
};

}  // namespace hydra

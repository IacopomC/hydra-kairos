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
#include <Eigen/Dense>
#include <cstdint>
#include <list>
#include <memory>
#include <set>
#include <unordered_map>

#include "hydra/reconstruction/voxel_types.h"

/**
 * @file gvd_graph.h
 * @brief Lightweight graph over Voronoi voxels. Each node records a position,
 *        clearance distance, basis-point count, and a set of sibling nodes,
 *        providing the intermediate connectivity that the place-graph
 *        extractors consume.
 */

namespace hydra::places {

/// @brief Per-node record in the GVD graph.
struct GvdMemberInfo {
  double distance = 0.0;          ///< Clearance to the nearest surface at this node (m).
  uint8_t num_basis_points = 0;   ///< Number of distinct nearest-surface parents supporting the node.
  Eigen::Vector3d position;       ///< World position of the node.
  GlobalIndex index;              ///< Voxel index this node was created from.
  std::set<uint64_t> siblings;    ///< Ids of adjacent nodes in the graph.
};

/// @brief Graph of GVD members keyed by integer id, with id recycling on removal.
class GvdGraph {
 public:
  using Ptr = std::shared_ptr<GvdGraph>;                          ///< Shared-pointer alias.
  using Nodes = std::unordered_map<uint64_t, GvdMemberInfo>;       ///< Map from node id to member info.

  GvdGraph();

  /// @brief Return whether the graph holds no nodes.
  bool empty() const;

  /// @brief Add a node at a position and voxel index.
  /// @return Id assigned to the new node (a recycled id when one is available).
  uint64_t addNode(const Eigen::Vector3d& position, const GlobalIndex& index);

  /// @brief Remove a node, detach it from its siblings, and recycle its id.
  void removeNode(uint64_t node);

  /// @brief Look up a node by id; throws if the id is not present.
  GvdMemberInfo* getNode(uint64_t node);

  /// @brief Const overload of getNode.
  const GvdMemberInfo* getNode(uint64_t node) const;

  /// @brief Access all nodes keyed by id.
  const Nodes& nodes() const;

  /// @brief Return whether a node with the given id exists.
  bool hasNode(uint64_t) const;

 protected:
  /// @brief Pop a recycled id if any are queued, otherwise allocate a fresh one.
  uint64_t getNextId();

 protected:
  uint64_t next_id_;               ///< Next never-used id to hand out.
  std::list<uint64_t> id_queue_;   ///< Freed ids awaiting reuse.

  Nodes nodes_;                    ///< Node storage keyed by id.
};

}  // namespace hydra::places

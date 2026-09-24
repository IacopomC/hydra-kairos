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

#include <kimera_pgmo/mesh_delta.h>

#include "hydra/common/dsg_types.h"

/**
 * @file place_mesh_connector.h
 * @brief Links place nodes to nearby mesh vertices and their deformation-graph
 *        indices using a spatial search over a mesh delta.
 */

namespace hydra {

/// @brief Connects each active place node to its nearest mesh vertices.
/// Builds a KD-tree over the active vertices of a mesh delta and, for every
/// place, resolves its stored voxel connections to mesh vertices, deformation
/// connections, and (when present) semantic labels.
class PlaceMeshConnector {
 public:
  using DeformationMapping = std::vector<size_t>;  ///< Maps local mesh indices to deformation-graph nodes.

  /// @brief Build the connector and its spatial index over the given mesh delta.
  /// @param delta Mesh delta whose active vertices are indexed; must be non-null.
  explicit PlaceMeshConnector(const kimera_pgmo::MeshDelta::Ptr& delta);
  ~PlaceMeshConnector();

  /// @brief Resolve mesh and deformation connections for each active place node.
  /// @param places Places layer whose active nodes are updated in place.
  /// @param mapping Per-vertex mapping from mesh index to deformation-graph node.
  /// @return Number of active places that had no voxel connections to resolve.
  size_t addConnections(const SceneGraphLayer& places,
                        const DeformationMapping& mapping) const;

 protected:
  kimera_pgmo::MeshDelta::Ptr delta_;  ///< Mesh delta backing the spatial index.

  struct Detail;                       ///< Hidden KD-tree and adaptor state.
  std::unique_ptr<Detail> internals_;  ///< Opaque spatial-search implementation.
};

}  // namespace hydra

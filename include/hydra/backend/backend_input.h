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
#include <spark_dsg/mesh.h>

#include <memory>
#include <sstream>

#include "hydra/common/robot_prefix_config.h"
#include "hydra/odometry/pose_graph_tracker.h"

/**
 * @file backend_input.h
 * @brief Input packet consumed by the backend on each step. Carries the latest
 *        deformation graph, agent pose updates, and mesh delta produced by the
 *        front end, along with per-vertex timestamp bookkeeping for the mesh.
 */

namespace hydra {

/// @brief Per-vertex first-seen and last-seen timestamps for a block of mesh
/// vertices, used to stamp the optimized mesh once it is updated.
struct StampUpdate {
  using Ptr = std::shared_ptr<StampUpdate>;

  /// @brief Allocates zero-initialized timestamp buffers for the given vertex count.
  /// @param num_vertices Number of vertices covered by this update.
  StampUpdate(size_t num_vertices)
      : first_seen(num_vertices, 0), last_seen(num_vertices, 0) {}

  /// @brief Writes the stored timestamps onto a contiguous block of mesh vertices.
  /// @param mesh Mesh to stamp; must carry timestamp and first-seen fields.
  /// @param vertex_start Index of the first vertex to write.
  void updateMesh(spark_dsg::Mesh& mesh, size_t vertex_start) const {
    if (!mesh.has_timestamps || !mesh.has_first_seen_stamps) {
      std::stringstream ss;
      ss << "bad mesh for stamp update! Required fields: " << std::boolalpha
         << "stamps=" << !mesh.has_timestamps
         << ", first_seen=" << !mesh.has_first_seen_stamps;
      throw std::runtime_error(ss.str());
    }

    if (vertex_start + first_seen.size() > mesh.numVertices()) {
      std::stringstream ss;
      ss << "bad mesh bounds for stamp update: [" << vertex_start << ", "
         << vertex_start + first_seen.size() << ") outside of total vertices "
         << mesh.numVertices();
      throw std::runtime_error(ss.str());
    }

    for (size_t i = 0; i < first_seen.size(); ++i) {
      mesh.first_seen_stamps[i + vertex_start] = first_seen[i];
      mesh.stamps[i + vertex_start] = last_seen[i];
    }
  }

  std::vector<spark_dsg::Mesh::Timestamp> first_seen;  ///< First-seen timestamp per vertex.
  std::vector<spark_dsg::Mesh::Timestamp> last_seen;   ///< Most recent timestamp per vertex.
};

/// @brief Bundle of front-end outputs the backend consumes on each step.
struct BackendInput {
  using Ptr = std::shared_ptr<BackendInput>;
  RobotPrefixConfig prefix;                       ///< Robot prefix that scopes the keys in this packet.
  uint64_t timestamp_ns;                          ///< Timestamp of this input (ns).
  uint64_t sequence_number;                       ///< Monotonic sequence counter for ordering.
  pose_graph_tools::PoseGraph deformation_graph;  ///< Deformation graph for mesh deformation.
  PoseGraphPacket agent_updates;                   ///< Incremental agent pose-graph measurements.
  kimera_pgmo::MeshDelta::Ptr mesh_update;         ///< Incremental change to the mesh.
  StampUpdate::Ptr mesh_stamp_update;              ///< Per-vertex timestamps accompanying the mesh delta.
};

}  // namespace hydra

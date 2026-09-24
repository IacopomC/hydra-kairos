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
#include "hydra/backend/backend_utilities.h"

#include <glog/logging.h>
#include <kimera_pgmo/mesh_delta.h>
#include <kimera_pgmo/sparse_keyframe.h>

#include <sstream>

namespace hydra::utils {

std::optional<uint64_t> getTimeNs(const DynamicSceneGraph& graph, gtsam::Symbol key) {
  NodeSymbol node(key.chr(), key.index());
  if (!graph.hasNode(node)) {
    LOG(ERROR) << "Missing node << " << node.getLabel() << "when logging loop closure";
    LOG(ERROR) << "Num dynamic nodes: " << graph.numDynamicNodes();
    return std::nullopt;
  }

  return graph.getNode(node).timestamp.value().count();
}

void updatePlace2dMesh(Place2dNodeAttributes& attrs,
                       const kimera_pgmo::MeshDelta& mesh_update,
                       const size_t num_archived_vertices) {
  size_t min_index = SIZE_MAX;
  size_t max_index = 0;
  auto iter = attrs.pcl_mesh_connections.begin();

  // Remap this place's mesh-vertex references through the latest delta: drop
  // references to deleted vertices and follow re-indexed (compacted) vertices.
  while (iter != attrs.pcl_mesh_connections.end()) {
    if (mesh_update.deleted_indices.count(*iter)) {
      iter = attrs.pcl_mesh_connections.erase(iter);
      continue;
    }

    auto map_iter = mesh_update.prev_to_curr.find(*iter);
    if (map_iter != mesh_update.prev_to_curr.end()) {
      *iter = map_iter->second;
    }
    min_index = std::min(min_index, *iter);
    max_index = std::max(max_index, *iter);
    ++iter;
  }
  attrs.pcl_min_index = min_index;
  attrs.pcl_max_index = max_index;

  // Once all referenced vertices fall below the archived watermark the place no
  // longer touches the active mesh and need not be deformed again.
  if (attrs.pcl_max_index < num_archived_vertices) {
    attrs.has_active_mesh_indices = false;
  }
}

void updatePlace2dBoundary(Place2dNodeAttributes& attrs,
                           const kimera_pgmo::MeshDelta& mesh_update) {
  const auto prev_boundary = attrs.boundary;
  const auto prev_boundary_connections = attrs.pcl_boundary_connections;
  attrs.boundary.clear();
  attrs.pcl_boundary_connections.clear();
  // Rebuild the boundary point list, dropping points whose backing vertex was
  // deleted and re-indexing the rest through the delta's remap.
  for (size_t i = 0; i < prev_boundary.size(); ++i) {
    if (mesh_update.deleted_indices.count(prev_boundary_connections.at(i))) {
      continue;
    }

    auto map_iter = mesh_update.prev_to_curr.find(prev_boundary_connections.at(i));
    if (map_iter != mesh_update.prev_to_curr.end()) {
      attrs.boundary.push_back(prev_boundary.at(i));
      attrs.pcl_boundary_connections.push_back(map_iter->second);
    } else {
      attrs.boundary.push_back(prev_boundary.at(i));
      attrs.pcl_boundary_connections.push_back(prev_boundary_connections.at(i));
    }
  }
}

void updatePlaces2d(SharedDsgInfo::Ptr dsg,
                    kimera_pgmo::MeshDelta& mesh_update,
                    size_t num_archived_vertices) {
  if (!dsg->graph->hasLayer(DsgLayers::MESH_PLACES)) {
    return;
  }
  for (auto& id_node_pair : dsg->graph->getLayer(DsgLayers::MESH_PLACES).nodes()) {
    auto& attrs = id_node_pair.second->attributes<spark_dsg::Place2dNodeAttributes>();
    if (!attrs.has_active_mesh_indices) {
      continue;
    }

    updatePlace2dMesh(attrs, mesh_update, num_archived_vertices);
    updatePlace2dBoundary(attrs, mesh_update);
  }
}

gtsam::Values getDenseFrames(const KeyMap& full_sparse_frame_map,
                             const FrameMap& sparse_frames,
                             const gtsam::Values& sparse_values) {
  // Was `empty() == 0`, which evaluated to "non-empty" and returned
  // sparse_values without interpolating — meaning dense (non-key)
  // agents were never updated when sparsification was on, and an
  // empty values map was returned (silent no-op) when it was off.
  if (full_sparse_frame_map.empty()) {
    LOG(INFO) << "[getDenseFrames] map empty -> returning sparse_values size="
              << sparse_values.size();
    return sparse_values;
  }

  gtsam::Values dense_values;
  size_t skipped_no_sparse = 0;
  size_t skipped_no_keyed = 0;
  for (const auto& [dense_key, sparse_key] : full_sparse_frame_map) {
    if (!sparse_values.exists(sparse_key)) {
      ++skipped_no_sparse;
      continue;
    }

    const auto sf_iter = sparse_frames.find(sparse_key);
    if (sf_iter == sparse_frames.end() ||
        sf_iter->second.keyed_transforms.find(dense_key) ==
            sf_iter->second.keyed_transforms.end()) {
      ++skipped_no_keyed;
      continue;
    }

    const auto& sparse_T_dense = sf_iter->second.keyed_transforms.at(dense_key);
    const auto agent_pose =
        sparse_values.at<gtsam::Pose3>(sparse_key).compose(sparse_T_dense);
    dense_values.insert(dense_key, agent_pose);
  }

  LOG(INFO) << "[getDenseFrames] map=" << full_sparse_frame_map.size()
            << " sparse_values=" << sparse_values.size()
            << " dense_out=" << dense_values.size()
            << " skipped(no_sparse_value)=" << skipped_no_sparse
            << " skipped(no_keyed_transform)=" << skipped_no_keyed;
  return dense_values;
}

}  // namespace hydra::utils

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
#include <optional>
#include <vector>

#include "hydra/common/dsg_types.h"
#include "hydra/reconstruction/voxel_types.h"

/**
 * @file mesh_utilities.h
 * @brief Helpers that derive node attributes and geometry from mesh vertices: centroids,
 *        object bounding boxes, the active subset of a mesh layer, and outlier-filtered
 *        bounding-box fitting.
 */

namespace hydra {

/// @brief Set a node's position to the centroid of the referenced mesh vertices.
/// Non-finite vertices are skipped.
/// @param mesh Mesh providing vertex positions.
/// @param indices Vertex indices to average.
/// @param attrs Node attributes whose position is updated.
/// @return True if at least one finite vertex contributed, false otherwise.
bool updateNodeCentroid(const spark_dsg::Mesh& mesh,
                        const std::vector<size_t>& indices,
                        NodeAttributes& attrs);

/// @brief Recompute an object's bounding box and centroid from its mesh vertices.
/// @param mesh Mesh providing vertex positions.
/// @param attrs Object attributes to update in place.
/// @param indices Vertices to use; if null, the object's stored mesh connections are used.
/// @param type Bounding box type to fit; if unset, the existing box type is kept.
/// @return True if the centroid update succeeded, false otherwise.
bool updateObjectGeometry(const spark_dsg::Mesh& mesh,
                          ObjectNodeAttributes& attrs,
                          const std::vector<size_t>* indices = nullptr,
                          std::optional<BoundingBox::Type> type = std::nullopt);

/// @brief Build a mesh layer holding only the updated blocks that are not archived.
/// @param mesh_layer Source mesh layer.
/// @param archived_blocks Blocks to exclude from the result.
/// @return A new layer with copies of the active updated blocks.
MeshLayer::Ptr getActiveMesh(const MeshLayer& mesh_layer,
                             const BlockIndices& archived_blocks);

/// @brief Fit a bounding box to a mesh after removing isolated outlier vertices.
/// Vertices with too few neighbors within the search radius are discarded before fitting.
/// @param mesh Mesh to fit.
/// @param type Bounding box type to produce.
/// @param inlier_min_neighbors Minimum neighbors within the radius to keep a vertex.
/// @param inlier_search_radius Neighbor search radius in meters.
/// @return The fitted box, or an empty box if no inliers remain.
BoundingBox fitBoxToFilteredMesh(const Mesh& mesh,
                                 BoundingBox::Type type = BoundingBox::Type::AABB,
                                 int inlier_min_neighbors = 5,
                                 double inlier_search_radius = 0.1);

}  // namespace hydra

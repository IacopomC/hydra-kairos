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

#include "hydra/places/gvd_voxel.h"

/**
 * @file gvd_parent_tracker.h
 * @brief Bookkeeping for the set of nearest-surface parents that make each
 *        Voronoi voxel multiply supported, plus a reference-counted cache of
 *        the surface vertices those parents correspond to.
 */

namespace hydra::places {

struct VoronoiCheckConfig;  ///< Forward declaration; full definition lives in the integrator config.

/// @brief Cached surface position of a parent voxel with a usage reference count.
struct GvdVertexInfo {
  Point pos;             ///< World position of the parent surface vertex.
  size_t ref_count = 0;  ///< Number of Voronoi voxels currently relying on this parent.
};

/// @brief Tracks the distinct parents of each Voronoi voxel and the surface
/// vertices they map to.
struct GvdParentTracker {
  /// @brief Record a neighbor's parent for a voxel if it is sufficiently distinct
  /// from the parents already stored.
  /// @return Count of distinct parents accumulated for the voxel after the update.
  uint8_t updateGvdParentMap(const GvdLayer& layer,
                             const VoronoiCheckConfig& config,
                             const GlobalIndex& voxel_index,
                             const GvdVoxel& neighbor);

  /// @brief Register a new parent vertex (or bump its reference count) using the
  /// parent voxel's surface position; ignores parents no longer on the surface.
  void markNewGvdParent(const GvdLayer& layer, const GlobalIndex& parent);

  /// @brief Drop a voxel's parents, decrementing the affected vertices' reference counts.
  void removeVoronoiFromGvdParentMap(const GlobalIndex& voxel_index);

  /// @brief Garbage-collect unreferenced or off-surface vertices and refresh the
  /// cached positions of the rest.
  void updateVertexMapping(const GvdLayer& layer);

  GlobalIndexMap<GlobalIndexSet> parents;          ///< Distinct parent indices per Voronoi voxel.
  GlobalIndexMap<GvdVertexInfo> parent_vertices;   ///< Reference-counted surface vertices per parent.
};

}  // namespace hydra::places

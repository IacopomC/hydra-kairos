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
#include <iostream>
#include <optional>

#include "hydra/reconstruction/voxel_types.h"

/**
 * @file gvd_voxel.h
 * @brief Per-voxel state for the generalized Voronoi diagram (GVD) and its
 *        underlying Euclidean signed distance field. Defines the GvdVoxel cell,
 *        the block and layer types built on it, and inline helpers for managing
 *        distance, parent (nearest-obstacle), and Voronoi-membership state.
 */

namespace hydra::places {

/// @brief Distance-field and Voronoi state stored at a single voxel.
struct GvdVoxel {
  float distance;            ///< Signed distance to the nearest surface (m); negative inside obstacles.
  bool observed = false;     ///< Whether the cell has ever received a measurement.
  bool fixed = false;        ///< Whether the distance comes directly from the truncated TSDF band and is held fixed.
  bool in_queue = false;     ///< Whether the cell is currently queued for ESDF propagation.
  bool to_raise = false;     ///< Whether the cell is flagged for a raise (distance increase) wavefront.
  bool is_negative = false;  ///< Whether the cell lies inside a surface (negative side).
  bool on_surface = false;   ///< Whether the cell sits on the surface and may act as a GVD parent.
  bool has_parent = false;   ///< Whether a nearest-surface parent voxel has been assigned.

  uint8_t num_extra_basis = 0;  ///< Count of distinct nearest-surface parents; nonzero marks a Voronoi cell.

  GlobalIndex parent;  ///< Global index of the nearest surface voxel (the parent).
  Point parent_pos;    ///< Cached world position of the parent, retained so block removal leaves no dangling reference.
};

/// @brief Voxel block specialized for GvdVoxel storage.
using GvdBlock = spatial_hash::VoxelBlock<GvdVoxel>;
/// @brief Spatially hashed layer of GvdBlock.
using GvdLayer = spatial_hash::VoxelLayer<GvdBlock>;

/// @brief Stream a human-readable summary of a voxel's state.
std::ostream& operator<<(std::ostream& out, const GvdVoxel& voxel);

/// @brief Clear a voxel's Voronoi membership by zeroing its parent count.
inline void resetVoronoi(GvdVoxel& voxel) { voxel.num_extra_basis = 0; }

/// @brief Return whether the voxel currently belongs to the Voronoi diagram.
inline bool isVoronoi(const GvdVoxel& voxel) { return voxel.num_extra_basis != 0; }

/// @brief Copy a nearest-surface parent onto a voxel from an adjacent ancestor.
/// If the ancestor already has a parent, that parent is inherited; otherwise the
/// ancestor itself becomes the parent.
/// @param voxel Voxel receiving the parent assignment.
/// @param ancestor Neighboring voxel whose parent (or own index) is propagated.
/// @param ancestor_index Global index of the ancestor voxel.
/// @param ancestor_pos World position of the ancestor voxel.
inline void setSdfParent(GvdVoxel& voxel,
                         const GvdVoxel& ancestor,
                         const GlobalIndex& ancestor_index,
                         const Point& ancestor_pos) {
  voxel.has_parent = true;
  if (ancestor.has_parent) {
    voxel.parent = ancestor.parent;
    voxel.parent_pos = ancestor.parent_pos;
  } else {
    voxel.parent = ancestor_index;
    voxel.parent_pos = ancestor_pos;
  }
}

/// @brief Drop a voxel's parent assignment.
inline void resetParent(GvdVoxel& voxel) { voxel.has_parent = false; }

/// @brief Set a voxel's distance to the default magnitude, signed by its side.
/// @param voxel Voxel to update.
/// @param default_distance Magnitude to assign; sign follows is_negative.
inline void setDefaultDistance(GvdVoxel& voxel, const double default_distance) {
  voxel.distance = std::copysign(default_distance, voxel.is_negative ? -1.0 : 1.0);
}

/// @brief Mark a voxel for a raise wavefront, clearing its parent and (unless
/// fixed) resetting its distance to the default.
/// @param voxel Voxel to flag for raising.
/// @param default_distance Distance assigned when the voxel is not fixed.
inline void setRaiseStatus(GvdVoxel& voxel, const double default_distance) {
  resetParent(voxel);
  voxel.to_raise = true;

  if (voxel.fixed) {
    return;
  }

  setDefaultDistance(voxel, default_distance);
}

/// @brief Mark a voxel as lying on the surface and clear its parent.
inline void setGvdSurfaceVoxel(GvdVoxel& voxel) {
  voxel.on_surface = true;
  resetParent(voxel);
}


}  // namespace hydra::places

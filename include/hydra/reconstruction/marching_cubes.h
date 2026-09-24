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

/**
 * @file marching_cubes.h
 * @brief Marching-cubes surface extraction: turns the eight TSDF corner samples
 *        of a voxel cube into mesh triangles by interpolating zero-crossings
 *        along the cube edges. Provides the per-corner sample type and the
 *        lookup tables that drive the triangulation.
 */

#include <spark_dsg/mesh.h>

#include <Eigen/Dense>
#include <array>
#include <optional>

#include "hydra/reconstruction/voxel_types.h"

namespace hydra {

struct OccupancyVoxel;

/// @brief One corner sample of a marching-cubes cube: the TSDF distance and
/// weight at a position, the surface color and optional semantic label, and
/// back-pointers to the voxels the resulting vertex came from.
struct SdfPoint {
  float distance;                ///< Signed distance at this corner.
  float weight;                  ///< TSDF weight at this corner.
  Eigen::Vector3f pos;           ///< World position of the corner.
  spark_dsg::Color color;        ///< Surface color at the corner.
  std::optional<uint32_t> label; ///< Optional semantic label.
  OccupancyVoxel* vertex_voxel = nullptr;        ///< Voxel that owns the emitted vertex.
  const TrackingVoxel* tracking_voxel = nullptr; ///< Free-space tracking voxel, if any.
};

/// @brief Stream a SdfPoint for debugging.
std::ostream& operator<<(std::ostream& out, const SdfPoint& point);

/// @brief Marching-cubes triangulation of a single voxel cube into mesh faces.
class MarchingCubes {
 public:
  using EdgePoints = std::array<SdfPoint, 12>;  ///< Interpolated points on the 12 cube edges.
  using EdgeStatus = std::array<uint8_t, 12>;   ///< Which edges have a surface crossing.
  using SdfPoints = std::array<SdfPoint, 8>;    ///< The 8 cube corner samples.

  /// @brief Interpolate the surface crossing on each cube edge that has one.
  /// @param points The 8 corner samples.
  /// @param edge_points Output interpolated point per crossing edge.
  /// @param edge_status Output flag per edge marking whether it crosses.
  /// @param min_sdf_difference Guard against dividing by a near-zero gradient.
  static void interpolateEdges(const SdfPoints& points,
                               EdgePoints& edge_points,
                               EdgeStatus& edge_status,
                               float min_sdf_difference = 1.0e-6);

  /// @brief Emit the triangles for one cube into @p mesh.
  /// @param block Block the cube belongs to.
  /// @param points The 8 corner samples.
  /// @param mesh Mesh to append the triangles to.
  /// @param compute_normals Whether to compute per-face normals.
  static void meshCube(const BlockIndex& block,
                       const SdfPoints& points,
                       spark_dsg::Mesh& mesh,
                       bool compute_normals = true);

  static const int kTriangleTable[256][16];  ///< Edge-to-triangle lookup per cube configuration.
  static const int kEdgeIndexPairs[12][2];   ///< The two corner indices bounding each edge.
};

}  // namespace hydra

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
 * @file mesh_integrator.h
 * @brief Builds the surface mesh from the TSDF. Runs marching cubes over the
 *        dirty blocks in parallel, handling each block's interior and the
 *        shared faces between neighbouring blocks, and writes the result into
 *        the map's mesh layer. Also defines the per-voxel occupancy bookkeeping
 *        that links a surface voxel to its emitted mesh vertex.
 */

#include "hydra/reconstruction/index_getter.h"
#include "hydra/reconstruction/mesh_integrator_config.h"
#include "hydra/reconstruction/voxel_types.h"

namespace hydra {

class VolumetricMap;

/// @brief Per-voxel meshing bookkeeping: whether the voxel lies on the surface,
/// and where its emitted vertex lives in the mesh. (Formerly VertexVoxel.)
struct OccupancyVoxel {
  bool on_surface = false;       ///< Whether the voxel contributed a surface vertex.
  size_t block_vertex_index;     ///< Index of that vertex within its mesh block.
  BlockIndex mesh_block;         ///< Mesh block holding the vertex.
};
using OccupancyBlock = spatial_hash::VoxelBlock<OccupancyVoxel>;  ///< Block of occupancy voxels.
using OccupancyLayer = spatial_hash::VoxelLayer<OccupancyBlock>;  ///< Layer of occupancy blocks.

/// @brief Extracts the surface mesh from a VolumetricMap's TSDF via marching cubes.
class MeshIntegrator {
 public:
  using BlockIndexGetter = IndexGetter<BlockIndex>;

  explicit MeshIntegrator(const MeshIntegratorConfig& config);

  virtual ~MeshIntegrator() = default;

  /// @brief Mesh the map's TSDF into its mesh layer.
  /// @param map Map whose TSDF is meshed and whose mesh layer is written.
  /// @param only_mesh_updated_blocks Mesh only blocks flagged mesh-dirty.
  /// @param clear_updated_flag Clear each block's mesh-dirty flag after meshing.
  /// @param occupancy Optional layer to record per-voxel vertex bookkeeping.
  virtual void generateMesh(VolumetricMap& map,
                            bool only_mesh_updated_blocks,
                            bool clear_updated_flag,
                            OccupancyLayer* occupancy = nullptr) const;

  /// @brief Allocate the mesh (and optional occupancy) blocks for @p blocks.
  void allocateBlocks(const BlockIndices& blocks,
                      VolumetricMap& map,
                      OccupancyLayer* occupancy) const;

  /// @brief Log how many blocks are being meshed, at the given verbosity.
  void showUpdateInfo(const VolumetricMap& map,
                      const BlockIndices& blocks,
                      int verbosity) const;

  /// @brief Mesh @p blocks across worker threads (interior or exterior pass).
  void launchThreads(const BlockIndices& blocks,
                     bool interior_pass,
                     VolumetricMap& map,
                     OccupancyLayer* occupancy) const;

  /// @brief Worker loop meshing block interiors pulled from @p index_getter.
  void processInterior(VolumetricMap* map,
                       BlockIndexGetter* index_getter,
                       OccupancyLayer* occupancy) const;

  /// @brief Worker loop meshing the shared faces between neighbouring blocks.
  void processExterior(VolumetricMap* map,
                       BlockIndexGetter* index_getter,
                       OccupancyLayer* occupancy) const;

  /// @brief Mesh one cube wholly inside a block.
  virtual void meshBlockInterior(const BlockIndex& block_index,
                                 const VoxelIndex& voxel_index,
                                 VolumetricMap& map,
                                 OccupancyLayer* occupancy) const;

  /// @brief Mesh one cube that straddles a block boundary, pulling corners from
  /// the neighbouring blocks.
  virtual void meshBlockExterior(const BlockIndex& block_index,
                                 const VoxelIndex& voxel_index,
                                 VolumetricMap& map,
                                 OccupancyLayer* occupancy) const;

  /// @brief Resolve the neighbouring block/corner for an out-of-range voxel index.
  static BlockIndex getNeighborIndex(const BlockIndex& block_idx,
                                     int voxels_per_side,
                                     VoxelIndex& corner_index);

  const MeshIntegratorConfig config;

 protected:
  const static Eigen::Matrix<int, 3, 8> cube_index_offsets_;
  mutable Eigen::Matrix<float, 3, 8> cube_coord_offsets_;
};

}  // namespace hydra

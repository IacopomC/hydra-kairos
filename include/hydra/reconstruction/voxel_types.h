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
 * @file voxel_types.h
 * @brief Core voxel, block, and layer types for the volumetric map. Defines the
 *        TSDF, semantic, and free-space-tracking voxels, the spatial-hash blocks
 *        that store them, the mesh block, and the layer aliases, and re-exports
 *        the spatial-hash geometry and index types used throughout the
 *        reconstruction code.
 */

#include <spark_dsg/mesh.h>
#include <spatial_hash/voxel_layer.h>

#include <cstddef>
#include <cstdint>

#include "hydra/common/common_types.h"

namespace hydra {

// Geometry types.
using spatial_hash::Point;

// Mesh types.
using spark_dsg::Mesh;

// Index types.
using spatial_hash::BlockIndex;
using spatial_hash::GlobalIndex;
using spatial_hash::VoxelIndex;
using spatial_hash::VoxelKey;

// Index containers.
using spatial_hash::BlockIndices;
using spatial_hash::GlobalIndices;
using spatial_hash::VoxelIndices;
using spatial_hash::VoxelKeys;

// Index Sets.
using BlockIndexSet = spatial_hash::IndexSet;
using VoxelIndexSet = spatial_hash::IndexSet;
using GlobalIndexSet = spatial_hash::IndexSet;

// Index hash maps.
template <typename ValueT>
using VoxelIndexMap = spatial_hash::IndexHashMap<ValueT>;
template <typename ValueT>
using BlockIndexMap = spatial_hash::IndexHashMap<ValueT>;
template <typename ValueT>
using GlobalIndexMap = spatial_hash::IndexHashMap<ValueT>;

// Voxel types.

/// @brief One TSDF voxel: the truncated signed distance to the nearest surface,
/// the accumulated integration weight, and an averaged surface color.
struct TsdfVoxel {
  float distance = 0.0f;   ///< Signed distance to the surface (metres).
  float weight = 0.0f;     ///< Accumulated integration weight.
  spark_dsg::Color color;  ///< Averaged surface color.
};

/// @brief One semantic voxel, based on the semantic voxel from Kimera-Semantics:
/// the current maximum-likelihood label and the log-likelihood scores backing it.
struct SemanticVoxel {
  uint32_t semantic_label = 0;  ///< Current MLE semantic label.
  // TODO(nathan) top-K!
  Eigen::VectorXf semantic_likelihoods;  ///< Per-label log-likelihood priors.
  bool empty = true;  ///< Whether the voxel has been initialized.
};

/// @brief One free-space tracking voxel: marks which parts of space are free with
/// high confidence, with the timestamps and flags the active window needs to manage it.
struct TrackingVoxel {
  TimeStamp first_observed = 0;  ///< When the voxel was first observed (ns).
  TimeStamp last_observed;       ///< When the voxel was last observed (ns).
  TimeStamp last_occupied;       ///< When the voxel was last seen occupied (ns).
  bool ever_free = false;        ///< Whether the voxel has ever been confidently free.
  bool active = false;           ///< Whether the voxel is in the active window.
  bool to_remove = false;        ///< Flagged for removal during maintenance.
};

// Block types.

/// @brief A block of TsdfVoxels on the spatial hash. Carries per-aspect dirty
/// flags (ESDF, mesh, tracking) on top of the base `updated` flag so each
/// downstream consumer can iterate over only the blocks that changed for it.
struct TsdfBlock : public spatial_hash::VoxelBlock<TsdfVoxel> {
  using Ptr = std::shared_ptr<TsdfBlock>;
  using ConstPtr = std::shared_ptr<const TsdfBlock>;

  TsdfBlock(const float voxel_size,
            const float voxels_per_side,
            const BlockIndex& index)
      : spatial_hash::VoxelBlock<TsdfVoxel>(voxel_size, voxels_per_side, index) {}

  mutable bool esdf_updated = false;      ///< Dirty for the ESDF/GVD consumer.
  mutable bool mesh_updated = false;      ///< Dirty for the mesh consumer.
  mutable bool tracking_updated = false;  ///< Dirty for the free-space tracker.

  /// @brief Mark the block dirty for every consumer.
  void setUpdated() const {
    updated = true;
    esdf_updated = true;
    mesh_updated = true;
    tracking_updated = true;
  }

  /// @brief Clear every dirty flag.
  void clearUpdated() const {
    updated = false;
    esdf_updated = false;
    mesh_updated = false;
    tracking_updated = false;
  }

  /// @brief Predicates for iterating over the blocks dirty for a given consumer.
  static bool esdfUpdated(const TsdfBlock& block) { return block.esdf_updated; }
  static bool meshUpdated(const TsdfBlock& block) { return block.mesh_updated; }
  static bool trackingUpdated(const TsdfBlock& block) { return block.tracking_updated; }
};

/// @brief A mesh block: a spatial-hash block that also is a mesh, holding the
/// surface geometry reconstructed within its spatial extent.
struct MeshBlock : public Mesh, public spatial_hash::Block {
  using Ptr = std::shared_ptr<MeshBlock>;
  using ConstPtr = std::shared_ptr<const MeshBlock>;
  MeshBlock(const float block_size, const BlockIndex& index, bool has_labels = false, bool has_stamps = false)
      : Mesh(true, has_stamps, has_labels, has_stamps), spatial_hash::Block(block_size, index) {}
};

/// @brief A block of TrackingVoxels on the spatial hash, used by the free-space tracker.
struct TrackingBlock : public spatial_hash::VoxelBlock<TrackingVoxel> {
  using Ptr = std::shared_ptr<TrackingBlock>;
  using ConstPtr = std::shared_ptr<const TrackingBlock>;
  TrackingBlock(const float voxel_size,
                const float voxels_per_side,
                const BlockIndex& index)
      : spatial_hash::VoxelBlock<TrackingVoxel>(voxel_size, voxels_per_side, index) {}
  bool has_active_data = false;  ///< Whether the block holds any active-window data.
};

/// @brief A block of SemanticVoxels on the spatial hash.
using SemanticBlock = spatial_hash::VoxelBlock<SemanticVoxel>;

// Layer types: a full map is one layer per aspect, each a spatial hash of blocks.
using TsdfLayer = spatial_hash::VoxelLayer<TsdfBlock>;          ///< TSDF distance/weight/color.
using SemanticLayer = spatial_hash::VoxelLayer<SemanticBlock>;  ///< Per-voxel semantics.
using MeshLayer = spatial_hash::BlockLayer<MeshBlock>;          ///< Reconstructed surface mesh.
using TrackingLayer = spatial_hash::VoxelLayer<TrackingBlock>;  ///< Free-space tracking.

}  // namespace hydra

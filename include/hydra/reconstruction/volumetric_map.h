// The contents of this file are originally from Panoptic-Mapping,
// under the following license:
//
// BSD 3-Clause License
// Copyright (c) 2021, ETHZ ASL
// All rights reserved.
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this
// list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// See https://github.com/ethz-asl/panoptic_mapping for original code and paper
//
// Modifications (including work done by Lukas Schmid for Khronos) fall under the same
// license as Hydra and are subject to the following copyright and disclaimer:
//
// Copyright 2022 Massachusetts Institute of Technology.
// All Rights Reserved
//
// Research was sponsored by the United States Air Force Research Laboratory and
// the United States Air Force Artificial Intelligence Accelerator and was
// accomplished under Cooperative Agreement Number FA8750-19-2-1000. The views
// and conclusions contained in this document are those of the authors and should
// not be interpreted as representing the official policies, either expressed or
// implied, of the United States Air Force or the U.S. Government. The U.S.
// Government is authorized to reproduce and distribute reprints for Government
// purposes notwithstanding any copyright notation herein.
#pragma once

/**
 * @file volumetric_map.h
 * @brief The volumetric map: the bundle of co-registered voxel layers (TSDF,
 *        optional semantics and free-space tracking) plus the surface mesh that
 *        together represent the reconstructed environment. Owns block allocation
 *        and removal across all layers, save/load, and cloning, and provides the
 *        tuple accessors that reach a block or voxel in every layer at once.
 */

#include "hydra/reconstruction/voxel_types.h"

namespace hydra {

/// @brief Merge all elements of a layer into another, overwriting overlapping
/// data. Assumes the layers share grid layout and block type.
/// @tparam Block Type of the block in both layers.
/// @param layer_in Input layer to merge.
/// @param layer_out Output layer to merge into.
template <typename Block>
void mergeLayer(const spatial_hash::BlockLayer<Block>& layer_in,
                spatial_hash::BlockLayer<Block>& layer_out) {
  for (const auto& block_in : layer_in) {
    layer_out.allocateBlock(block_in.index) = block_in;
  }
}
template <typename Block>
void mergeLayer(const spatial_hash::VoxelLayer<Block>& layer_in,
                spatial_hash::VoxelLayer<Block>& layer_out) {
  for (const auto& block_in : layer_in) {
    layer_out.allocateBlock(block_in.index) = block_in;
  }
}

/// @brief Pointers to the same voxel across every layer (null where a layer is
/// absent), so a single linear index can be read/written in all layers at once.
struct VoxelTuple {
  TsdfVoxel* tsdf = nullptr;
  SemanticVoxel* semantic = nullptr;
  TrackingVoxel* tracking = nullptr;
};

/// @brief The co-located blocks across every layer for one block index.
/// getVoxels() gathers the per-layer voxels at a linear index within the block.
struct BlockTuple {
  TsdfBlock::Ptr tsdf;
  SemanticBlock::Ptr semantic;
  TrackingBlock::Ptr tracking;
  VoxelTuple getVoxels(const size_t linear_index) const;
};

/// @brief The reconstructed map: a TSDF layer plus a surface mesh, with optional
/// semantic and free-space-tracking layers, all sharing one block grid. Handles
/// block allocation/removal across the layers, persistence, and cloning.
class VolumetricMap {
 public:
  struct Config {
    /// Voxel size (metres).
    float voxel_size = 0.1f;
    /// Number of voxels per block side.
    size_t voxels_per_side = 16;
    /// TSDF truncation distance (metres).
    float truncation_distance = 0.3f;
    /// Whether the map has a semantic layer.
    bool with_semantics = false;
    /// Whether the map has a free-space tracking layer.
    bool with_tracking = false;
  } const config;

  explicit VolumetricMap(const Config& config);
  virtual ~VolumetricMap() = default;

  /// @brief Edge length of one block (voxel_size * voxels_per_side), in metres.
  float blockSize() const { return config.voxel_size * config.voxels_per_side; }

  /// @brief Gather the co-located blocks at @p index across every layer.
  virtual BlockTuple getBlock(const BlockIndex& index);

  /// @brief Access the individual layers. The semantic and tracking layers are
  /// null when the map was configured without them.
  TsdfLayer& getTsdfLayer() { return tsdf_layer_; }
  const TsdfLayer& getTsdfLayer() const { return tsdf_layer_; }

  MeshLayer& getMeshLayer() { return mesh_layer_; }
  const MeshLayer& getMeshLayer() const { return mesh_layer_; }

  SemanticLayer* getSemanticLayer() { return semantic_layer_.get(); }
  const SemanticLayer* getSemanticLayer() const { return semantic_layer_.get(); }

  TrackingLayer* getTrackingLayer() { return tracking_layer_.get(); }
  const TrackingLayer* getTrackingLayer() const { return tracking_layer_.get(); }

  /// @brief Whether the optional semantic / tracking layers are present.
  bool hasSemantics() const { return semantic_layer_ != nullptr; }

  bool hasTracking() const { return tracking_layer_ != nullptr; }

  /// @brief Allocate a block in all relevant layers of the map.
  /// @param index Index of the block to allocate.
  /// @return True if a new block was allocated, false if it already existed.
  virtual bool allocateBlock(const BlockIndex& index);

  /// @brief Allocate a set of blocks in all relevant layers of the map.
  /// @param blocks Block indices to allocate.
  /// @return The block indices that were newly allocated.
  BlockIndices allocateBlocks(const BlockIndices& blocks);

  /// @brief Remove a block (or a set of blocks) from every layer.
  virtual void removeBlock(const BlockIndex& block);

  void removeBlocks(const BlockIndices& blocks);

  /// @brief Human-readable summary of layer/block/voxel counts, for logging.
  virtual std::string printStats() const;

  /// @brief Footprint of the windowed map's voxel layers, by allocated-block
  /// count and bytes. Because the layers are bounded by the active window, the
  /// block counts stay flat as the explored area grows — the direct signal of
  /// the bounded-memory property. The GVD/places layer is owned elsewhere and
  /// is not included here.
  struct MemoryStats {
    size_t voxels_per_side = 0;
    size_t tsdf_blocks = 0;
    size_t semantic_blocks = 0;  ///< 0 when the map has no semantic layer.
    /// tsdf_bytes is exact (TsdfVoxel is POD). semantic_bytes counts the voxel
    /// containers only and excludes each voxel's variable-length likelihood
    /// heap, so it is a lower bound when semantics are enabled.
    size_t tsdf_bytes = 0;
    size_t semantic_bytes = 0;
  };
  MemoryStats memoryStats() const;

  /// @brief Serialize the whole map to @p filepath.
  void save(const std::string& filepath) const;

  /// @brief Build a map from an existing TSDF layer, re-deriving the config from it.
  static std::unique_ptr<VolumetricMap> fromTsdf(const TsdfLayer& tsdf,
                                                 double truncation_distance_m,
                                                 bool with_semantics = false);

  /// @brief Load a map previously written by save().
  static std::unique_ptr<VolumetricMap> load(const std::string& filepath);

  /// @brief Deep-copy the whole map.
  virtual std::unique_ptr<VolumetricMap> clone() const;

  /// @brief Deep-copy only the blocks currently flagged updated.
  virtual std::unique_ptr<VolumetricMap> cloneUpdated() const;

  /// @brief Merge another map's blocks into this one (overwriting overlaps).
  virtual void updateFrom(const VolumetricMap& other);

 protected:
  TsdfLayer tsdf_layer_;
  MeshLayer mesh_layer_;
  SemanticLayer::Ptr semantic_layer_;
  TrackingLayer::Ptr tracking_layer_;
};

void declare_config(VolumetricMap::Config& config);

}  // namespace hydra

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
#include "hydra/reconstruction/volumetric_map.h"

#include <config_utilities/config_utilities.h>
#include <config_utilities/parsing/yaml.h>

#include <sstream>

#include "hydra/utils/display_utilities.h"
#include "hydra/utils/layer_io.h"

namespace hydra {

void declare_config(VolumetricMap::Config& config) {
  using namespace config;
  name("VolumetricMap");
  field(config.voxel_size, "voxel_size", "m");
  field(config.voxels_per_side, "voxels_per_side");
  field(config.truncation_distance, "truncation_distance", "m");
  field(config.with_semantics, "with_semantics");
  field(config.with_tracking, "with_tracking");

  check(config.voxel_size, GT, 0, "voxel_size");
  check(config.voxels_per_side, GT, 0, "voxels_per_side");
  check(config.truncation_distance, GT, 0, "truncation_distance");
}

VolumetricMap::VolumetricMap(const Config& _config)
    : config(config::checkValid(_config)),
      tsdf_layer_(config.voxel_size, config.voxels_per_side),
      mesh_layer_(tsdf_layer_.blockSize()) {
  if (config.with_semantics) {
    semantic_layer_.reset(new SemanticLayer(config.voxel_size, config.voxels_per_side));
  }

  if (config.with_tracking) {
    tracking_layer_.reset(new TrackingLayer(config.voxel_size, config.voxels_per_side));
  }
}

bool VolumetricMap::allocateBlock(const BlockIndex& index) {
  if (tsdf_layer_.hasBlock(index)) {
    return false;
  }
  tsdf_layer_.allocateBlock(index);
  // NOTE: the mesh block is intentionally not allocated here; the mesh integrator does.
  if (semantic_layer_) {
    semantic_layer_->allocateBlock(index);
  }
  if (tracking_layer_) {
    tracking_layer_->allocateBlock(index);
  }
  return true;
}

BlockIndices VolumetricMap::allocateBlocks(const BlockIndices& blocks) {
  BlockIndices new_blocks;
  for (const auto& idx : blocks) {
    if (allocateBlock(idx)) {
      new_blocks.push_back(idx);
    }
  }
  return new_blocks;
}

void VolumetricMap::removeBlock(const BlockIndex& block_index) {
  tsdf_layer_.removeBlock(block_index);
  mesh_layer_.removeBlock(block_index);
  if (semantic_layer_) {
    semantic_layer_->removeBlock(block_index);
  }
  if (tracking_layer_) {
    tracking_layer_->removeBlock(block_index);
  }
}

void VolumetricMap::removeBlocks(const BlockIndices& blocks) {
  for (const auto& idx : blocks) {
    removeBlock(idx);
  }
}

BlockTuple VolumetricMap::getBlock(const BlockIndex& index) {
  BlockTuple tuple;
  tuple.tsdf = tsdf_layer_.getBlockPtr(index);
  if (semantic_layer_) {
    tuple.semantic = semantic_layer_->getBlockPtr(index);
  }
  if (tracking_layer_) {
    tuple.tracking = tracking_layer_->getBlockPtr(index);
  }
  return tuple;
}

VoxelTuple BlockTuple::getVoxels(const size_t linear_index) const {
  VoxelTuple tuple;
  if (tsdf) {
    tuple.tsdf = &tsdf->getVoxel(linear_index);
  }
  if (semantic) {
    tuple.semantic = &semantic->getVoxel(linear_index);
  }
  if (tracking) {
    tuple.tracking = &tracking->getVoxel(linear_index);
  }
  return tuple;
}

void VolumetricMap::save(const std::string& filepath) const {
  // TODO: consider hdf5 or something
  // TODO: could use binary serialization tools to write a proper (single) file io

  // Write the map config plus a semantics flag to a sidecar YAML, then dump each layer
  // to its own file. Mesh and tracking layers are derived/transient and not persisted.
  YAML::Node config_node;
  config_node["map"] = config::toYaml(config);
  config_node["has_semantics"] = hasSemantics();
  std::ofstream config_file(filepath + ".yaml");
  config_file << config_node;

  io::saveLayer<TsdfLayer>(filepath + "_tsdf", tsdf_layer_);
  if (semantic_layer_) {
    io::saveLayer<SemanticLayer>(filepath + "_semantics", *semantic_layer_);
  }
}

std::unique_ptr<VolumetricMap> VolumetricMap::load(const std::string& filepath) {
  auto tsdf = io::loadLayer<TsdfLayer>(filepath + "_tsdf");
  if (!tsdf) {
    return nullptr;
  }

  // Cross-check the loaded TSDF geometry against the saved config; a mismatch means the
  // files are inconsistent and the map cannot be reconstructed safely.
  const auto cpath = filepath + ".yaml";
  auto config = config::fromYamlFile<VolumetricMap::Config>(cpath, "map");
  if (std::abs(config.voxel_size - tsdf->voxel_size) > 1.0e-5) {
    LOG(ERROR) << "TSDF voxel size does not match config voxel size";
    return nullptr;
  }

  if (config.voxels_per_side != tsdf->voxels_per_side) {
    LOG(ERROR) << "TSDF vps does not match config vps";
    return nullptr;
  }

  const auto node = YAML::LoadFile(cpath);
  if (node["has_semantics"] && node["has_semantics"].as<bool>()) {
    config.with_semantics = true;
  }

  auto map = std::make_unique<VolumetricMap>(config);
  map->tsdf_layer_ = *tsdf;
  if (config.with_semantics) {
    map->semantic_layer_ = io::loadLayer<SemanticLayer>(filepath + "_semantics");
  }

  return map;
}

VolumetricMap::MemoryStats VolumetricMap::memoryStats() const {
  MemoryStats stats;
  stats.voxels_per_side = tsdf_layer_.voxels_per_side;
  const size_t voxels_per_block =
      stats.voxels_per_side * stats.voxels_per_side * stats.voxels_per_side;
  stats.tsdf_blocks = tsdf_layer_.numBlocks();
  stats.tsdf_bytes = stats.tsdf_blocks * voxels_per_block * sizeof(TsdfVoxel);
  if (semantic_layer_) {
    stats.semantic_blocks = semantic_layer_->numBlocks();
    stats.semantic_bytes =
        stats.semantic_blocks * voxels_per_block * sizeof(SemanticVoxel);
  }
  return stats;
}

std::string VolumetricMap::printStats() const {
  const auto stats = memoryStats();
  std::stringstream ss;
  ss << "TSDF: " << stats.tsdf_blocks << " blocks, "
     << getHumanReadableMemoryString(stats.tsdf_bytes);
  if (semantic_layer_) {
    ss << "; SEMANTICS: " << stats.semantic_blocks << " blocks, >="
       << getHumanReadableMemoryString(stats.semantic_bytes);
  }
  return ss.str();
}

std::unique_ptr<VolumetricMap> VolumetricMap::fromTsdf(const TsdfLayer& tsdf,
                                                       double truncation_distance_m,
                                                       bool with_semantics) {
  VolumetricMap::Config config;
  config.voxel_size = tsdf.voxel_size;
  config.voxels_per_side = tsdf.voxels_per_side;
  config.truncation_distance = truncation_distance_m;
  config.with_semantics = with_semantics;
  auto to_return = std::make_unique<VolumetricMap>(config);
  to_return->tsdf_layer_ = tsdf;
  return to_return;
}

std::unique_ptr<VolumetricMap> VolumetricMap::clone() const {
  return std::make_unique<VolumetricMap>(*this);
}

std::unique_ptr<VolumetricMap> VolumetricMap::cloneUpdated() const {
  // Build a partial copy containing only the blocks flagged as updated, so consumers can
  // process recent changes without copying the entire map.
  auto map = std::make_unique<VolumetricMap>(config);
  const auto blocks = tsdf_layer_.blockIndicesWithCondition(
      [](const auto& block) { return block.updated; });
  for (const auto& idx : blocks) {
    map->tsdf_layer_.allocateBlock(idx) = tsdf_layer_.getBlock(idx);
    if (mesh_layer_.hasBlock(idx)) {
      map->mesh_layer_.allocateBlock(idx) = mesh_layer_.getBlock(idx);
    }

    if (semantic_layer_) {
      // other semantic layer is always allocated if this semantic layer exists
      map->semantic_layer_->allocateBlock(idx) = semantic_layer_->getBlock(idx);
    }

    if (tracking_layer_) {
      // other tracking layer is always allocated if this tracking layer exists
      map->tracking_layer_->allocateBlock(idx) = tracking_layer_->getBlock(idx);
    }
  }

  return map;
}

void VolumetricMap::updateFrom(const VolumetricMap& other) {
  const auto has_semantics =
      semantic_layer_ != nullptr && other.semantic_layer_ != nullptr;

  mergeLayer(other.tsdf_layer_, tsdf_layer_);
  mergeLayer(other.mesh_layer_, mesh_layer_);

  if (has_semantics) {
    mergeLayer(*other.semantic_layer_, *semantic_layer_);
  }
}

}  // namespace hydra

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

#include <spark_dsg/serialization/binary_conversions.h>
#include <spark_dsg/serialization/binary_serialization.h>

#include <fstream>
#include <string>
#include <typeinfo>
#include <vector>

#include "hydra/places/gvd_voxel.h"
#include "hydra/reconstruction/voxel_types.h"

/**
 * @file layer_io.h
 * @brief Binary serialization for voxel layers. Templated helpers write and read voxels,
 *        blocks and whole layers, with per-type specializations for the TSDF, semantic
 *        and GVD voxel formats. Public save and load entry points handle the on-disk
 *        file, prepending a layer-type tag so loads can be validated against the
 *        requested layer type.
 */

namespace hydra::io {

namespace internal {

using spark_dsg::serialization::BinaryDeserializer;
using spark_dsg::serialization::BinarySerializer;
using spatial_hash::VoxelLayer;

/// @brief Tag identifying which voxel layer format a serialized stream holds.
enum class LayerType : uint8_t {
  INVALID,   ///< Unknown or unsupported layer type.
  TSDF,      ///< Truncated signed distance field layer.
  SEMANTIC,  ///< Semantic label layer.
  GVD        ///< Generalized Voronoi diagram layer.
};

/// @brief Map a layer type to its LayerType tag at compile time.
/// @tparam LayerT Layer type to classify.
/// @return The matching tag, or LayerType::INVALID if unsupported.
template <typename LayerT>
LayerType getLayerType() {
  if constexpr (std::is_same_v<LayerT, TsdfLayer>) {
    return LayerType::TSDF;
  } else if constexpr (std::is_same_v<LayerT, SemanticLayer>) {
    return LayerType::SEMANTIC;
  } else if constexpr (std::is_same_v<LayerT, places::GvdLayer>) {
    return LayerType::GVD;
  }
  return LayerType::INVALID;
}

/// @brief Render a layer type tag as a lowercase name for logging.
inline std::string toString(LayerType type) {
  switch (type) {
    case LayerType::TSDF:
      return "tsdf";
    case LayerType::SEMANTIC:
      return "semantic";
    case LayerType::GVD:
      return "gvd";
    default:
      return "invalid";
  }
}

/// @brief Write a single voxel to the serializer.
/// The unspecialized template logs an error and fails; each supported voxel type
/// provides its own specialization.
/// @return True on success, false if the voxel type has no serialization.
template <typename VoxelT>
bool serializeVoxel(BinarySerializer& /* serializer */, const VoxelT& /* voxel */) {
  LOG(ERROR) << "No serialization function for voxel type " << typeid(VoxelT).name()
             << ".";
  return false;
}

/// @brief Read a single voxel from the deserializer.
/// The unspecialized template logs an error and fails; each supported voxel type
/// provides its own specialization.
/// @return True on success, false if the voxel type has no deserialization.
template <typename VoxelT>
bool deserializeVoxel(BinaryDeserializer& /* deserializer */, VoxelT& /* voxel */) {
  LOG(ERROR) << "No deserialization function for voxel type " << typeid(VoxelT).name()
             << ".";
  return false;
}

template <>
inline bool serializeVoxel(BinarySerializer& serializer, const TsdfVoxel& voxel) {
  serializer.write(voxel.distance);
  serializer.write(voxel.weight);
  serializer.write(voxel.color);
  return true;
}

template <>
inline bool deserializeVoxel(BinaryDeserializer& deserializer, TsdfVoxel& voxel) {
  deserializer.read(voxel.distance);
  deserializer.read(voxel.weight);
  deserializer.read(voxel.color);
  return true;
}

template <>
inline bool serializeVoxel(BinarySerializer& serializer, const SemanticVoxel& voxel) {
  serializer.write(voxel.semantic_label);
  serializer.write(voxel.semantic_likelihoods);
  serializer.write(voxel.empty);
  return true;
}

template <>
inline bool deserializeVoxel(BinaryDeserializer& deserializer, SemanticVoxel& voxel) {
  deserializer.read(voxel.semantic_label);
  deserializer.read(voxel.semantic_likelihoods);
  deserializer.read(voxel.empty);
  return true;
}

template <>
inline bool serializeVoxel(BinarySerializer& serializer,
                           const places::GvdVoxel& voxel) {
  serializer.write(voxel.distance);
  serializer.write(voxel.observed);
  serializer.write(voxel.fixed);
  serializer.write(voxel.in_queue);
  serializer.write(voxel.to_raise);
  serializer.write(voxel.is_negative);
  serializer.write(voxel.on_surface);
  serializer.write(voxel.has_parent);
  serializer.write(voxel.num_extra_basis);
  serializer.write(voxel.parent);
  serializer.write(voxel.parent_pos);
  return true;
}

template <>
inline bool deserializeVoxel(BinaryDeserializer& deserializer,
                             places::GvdVoxel& voxel) {
  deserializer.read(voxel.distance);
  deserializer.read(voxel.observed);
  deserializer.read(voxel.fixed);
  deserializer.read(voxel.in_queue);
  deserializer.read(voxel.to_raise);
  deserializer.read(voxel.is_negative);
  deserializer.read(voxel.on_surface);
  deserializer.read(voxel.has_parent);
  deserializer.read(voxel.num_extra_basis);
  deserializer.read(voxel.parent);
  deserializer.read(voxel.parent_pos);
  return true;
}

/// @brief Write a block's index, update flag and every voxel it contains.
/// @return True on success, false if a voxel failed to serialize.
template <typename BlockT>
bool serializeBlock(BinarySerializer& serializer, const BlockT& block) {
  // Block config.
  serializer.write(block.index);
  serializer.write(block.updated);

  // Write voxels.
  for (const auto& voxel : block) {
    if (!serializeVoxel(serializer, voxel)) {
      LOG(ERROR) << "Failed to serialize voxel.";
      return false;
    }
  }
  return true;
}

/// @brief Read a block from the stream and allocate it into the layer.
/// @return True on success, false if a voxel failed to deserialize.
template <typename BlockT>
bool deserializeBlock(BinaryDeserializer& deserializer, VoxelLayer<BlockT>& layer) {
  // Block config.
  BlockIndex index;
  deserializer.read(index);
  auto& block = layer.allocateBlock(index);
  deserializer.read(block.updated);

  // Create block.
  for (auto& voxel : block) {
    if (!deserializeVoxel(deserializer, voxel)) {
      LOG(ERROR) << "Failed to deserialize voxel.";
      return false;
    }
  }
  return true;
}

/// @brief Block serialization specialization for TsdfBlock, which carries extra mesh and
/// ESDF update flags beyond the base block fields.
template <>
inline bool serializeBlock(BinarySerializer& serializer, const TsdfBlock& block) {
  if (!serializeBlock<spatial_hash::VoxelBlock<TsdfVoxel>>(serializer, block)) {
    return false;
  }
  serializer.write(block.esdf_updated);
  serializer.write(block.mesh_updated);
  return true;
}

/// @brief Block deserialization specialization for TsdfBlock, reading the extra mesh and
/// ESDF update flags after the base block fields.
template <>
inline bool deserializeBlock(BinaryDeserializer& deserializer,
                             VoxelLayer<TsdfBlock>& layer) {
  // Block config.
  BlockIndex index;
  deserializer.read(index);
  auto& block = layer.allocateBlock(index);
  deserializer.read(block.updated);

  // Create block.
  for (auto& voxel : block) {
    if (!deserializeVoxel(deserializer, voxel)) {
      LOG(ERROR) << "Failed to deserialize voxel.";
      return false;
    }
  }
  deserializer.read(block.esdf_updated);
  deserializer.read(block.mesh_updated);
  return true;
}

/// @brief Write a whole voxel layer: its type tag, voxel geometry, block count and every
/// block.
/// @return True on success, false if the layer type is unsupported or a block fails.
template <typename LayerT>
bool serializeLayer(BinarySerializer& serializer, const LayerT& layer) {
  // Layer config.
  const auto type = getLayerType<LayerT>();
  if (type == LayerType::INVALID) {
    LOG(ERROR) << "Invalid block type " << typeid(LayerT).name()
               << " for serialization.";
    return false;
  }
  serializer.write(static_cast<uint8_t>(type));
  serializer.write(layer.voxel_size);
  serializer.write(layer.voxels_per_side);
  serializer.write(layer.numBlocks());

  // Write blocks.
  for (const auto& block : layer) {
    if (!serializeBlock(serializer, block)) {
      LOG(ERROR) << "Failed to serialize block " << block.index.transpose() << ".";
      return false;
    }
  }
  return true;
}

/// @brief Read a whole voxel layer, validating that the stored type tag matches LayerT.
/// @return The reconstructed layer, or nullptr on a type mismatch or read failure.
template <typename LayerT>
typename LayerT::Ptr deserializeLayer(BinaryDeserializer& deserializer) {
  // Layer config.
  const auto expected_type = getLayerType<LayerT>();
  if (expected_type == LayerType::INVALID) {
    LOG(ERROR) << "Invalid block type " << typeid(LayerT).name()
               << " for deserialization.";
    return nullptr;
  }
  uint8_t type_int;
  deserializer.read(type_int);
  const auto type = static_cast<LayerType>(type_int);
  if (type == LayerType::INVALID) {
    LOG(ERROR) << "Invalid layer type in saved file.";
    return nullptr;
  }
  if (type != getLayerType<LayerT>()) {
    LOG(ERROR) << "Layer type mismatch. Expected " << toString(expected_type)
               << " but read " << toString(type) << ".";
    return nullptr;
  }
  float voxel_size;
  deserializer.read(voxel_size);
  size_t voxels_per_side;
  deserializer.read(voxels_per_side);
  size_t num_blocks;
  deserializer.read(num_blocks);

  // Create layer.
  auto layer = std::make_shared<LayerT>(voxel_size, voxels_per_side);
  for (size_t i = 0; i < num_blocks; ++i) {
    if (!deserializeBlock<typename LayerT::BlockType>(deserializer, *layer)) {
      LOG(ERROR) << "Failed to deserialize block " << i << ".";
      return nullptr;
    }
  }
  return layer;
}

}  // namespace internal

/// @brief Write a voxel layer to a file in binary form.
/// A ".layer" extension is appended when the path has none. Mesh layers are not voxel
/// layers and are not handled here.
/// @tparam LayerT The voxel layer type being saved.
/// @param filepath The file to write to, including full path and optionally extension.
/// @param layer The layer to save.
/// @return True if the layer was saved successfully.
template <typename LayerT>
bool saveLayer(std::string filepath,
               const spatial_hash::VoxelLayer<typename LayerT::BlockType>& layer) {
  // Fix extension if needed.
  if (filepath.find('.') == std::string::npos) {
    filepath += ".layer";
  }

  // Setup file.
  std::ofstream out(filepath, std::ios::out | std::ios::binary);
  if (!out.is_open()) {
    LOG(ERROR) << "Could not open file " << filepath << " for writing.";
    return false;
  }

  // Save.
  std::vector<uint8_t> buffer;
  spark_dsg::serialization::BinarySerializer serializer(&buffer);
  if (!internal::serializeLayer(serializer, layer)) {
    LOG(ERROR) << "Failed to serialize layer.";
    return false;
  }
  out.write(reinterpret_cast<const char*>(buffer.data()), buffer.size());
  return true;
}

/// @brief Load a voxel layer from a binary file.
/// A ".layer" extension is appended when the path has none. Mesh layers are not voxel
/// layers and are not handled here.
/// @tparam LayerT The voxel layer type expected in the file.
/// @param filepath The file to read from, including full path and optionally extension.
/// @return The loaded layer, or nullptr if the file could not be read.
template <typename LayerT>
std::shared_ptr<spatial_hash::VoxelLayer<typename LayerT::BlockType>> loadLayer(
    std::string filepath) {
  // Fix extension if needed.
  if (filepath.find('.') == std::string::npos) {
    filepath += ".layer";
  }

  // Setup file.
  std::ifstream in(filepath, std::ios::in | std::ios::binary);
  if (!in.is_open()) {
    LOG(ERROR) << "Could not open file " << filepath << " for reading.";
    return nullptr;
  }

  // Read file.
  std::vector<uint8_t> buffer(std::istreambuf_iterator<char>(in), {});
  spark_dsg::serialization::BinaryDeserializer deserializer(buffer);
  return internal::deserializeLayer<LayerT>(deserializer);
}

}  // namespace hydra::io

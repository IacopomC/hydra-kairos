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

#include <kimera_pgmo/utils/mesh_interface.h>

#include "hydra/reconstruction/voxel_types.h"

/**
 * @file pgmo_mesh_interface.h
 * @brief Adapters that expose hydra mesh representations through the mesh-deformation
 *        library's MeshInterface. One adapter wraps a block-structured mesh layer, the
 *        other a flat mesh, so both can supply vertices, semantics and block indices to
 *        the deformation routines.
 */

namespace hydra {

/// @brief Adapts a block-structured mesh layer to the deformation MeshInterface.
struct PgmoMeshLayerInterface : public kimera_pgmo::MeshInterface {
  /// @brief Wrap the given mesh layer.
  PgmoMeshLayerInterface(const MeshLayer& mesh);

  /// @brief Return the indices of all blocks in the mesh.
  const BlockIndices& blockIndices() const override;

  /// @brief Select the block whose vertices subsequent queries operate on.
  void markBlockActive(const BlockIndex& block) const override;

  /// @brief Number of vertices in the currently active block.
  size_t activeBlockSize() const override;

  /// @brief Return a vertex of the active block as a colored point.
  pcl::PointXYZRGBA getActiveVertex(size_t index) const override;

  /// @brief Report whether the mesh carries per-vertex semantic labels.
  bool hasSemantics() const override;

  /// @brief Return the semantic label of an active-block vertex, if any.
  std::optional<uint32_t> getActiveSemantics(size_t index) const override;

  /// @brief Produce an owning copy of this interface.
  kimera_pgmo::MeshInterface::Ptr clone() const override;

 private:
  const MeshLayer& mesh_;                  ///< Wrapped mesh layer.
  BlockIndices block_indices_;             ///< Cached list of block indices.
  mutable MeshBlock::ConstPtr active_mesh_;  ///< Block selected by markBlockActive().
};

/// @brief Adapts a flat scene-graph mesh to the deformation MeshInterface.
/// The whole mesh is treated as a single active block, so markBlockActive() is a no-op.
struct PgmoMeshInterface : public kimera_pgmo::MeshInterface {
  /// @brief Wrap the given mesh.
  PgmoMeshInterface(const Mesh& mesh);

  /// @brief Return the single block index representing the whole mesh.
  const BlockIndices& blockIndices() const override;

  /// @brief No-op: this mesh exposes only one implicit block.
  void markBlockActive(const BlockIndex&) const override {}

  /// @brief Total number of vertices in the mesh.
  size_t activeBlockSize() const override;

  /// @brief Return a vertex as a colored point.
  pcl::PointXYZRGBA getActiveVertex(size_t index) const override;

  /// @brief Report whether the mesh carries per-vertex semantic labels.
  bool hasSemantics() const override;

  /// @brief Return the semantic label of a vertex, if any.
  std::optional<uint32_t> getActiveSemantics(size_t index) const override;

  /// @brief Produce an owning copy of this interface.
  kimera_pgmo::MeshInterface::Ptr clone() const override;

 private:
  const Mesh& mesh_;            ///< Wrapped mesh.
  BlockIndices block_indices_;  ///< Single block index covering the mesh.
};

}  // namespace hydra

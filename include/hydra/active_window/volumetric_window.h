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

#include <spatial_hash/block.h>
#include <spatial_hash/types.h>

#include <Eigen/Geometry>

/**
 * @file volumetric_window.h
 * @brief The window policy that decides which map blocks stay active: it tests
 *        each block against the window and archives those that fall outside.
 */

namespace hydra {

class VolumetricMap;

/// @brief Lightweight descriptor of a map block for in-bounds testing.
struct VolumetricBlockInfo {
  const spatial_hash::BlockIndex index;  ///< Block index in the spatial hash.
  const double block_size;               ///< Edge length of the block (m).
  const uint64_t update_stamp_ns;        ///< Timestamp the block was last updated (ns).

  VolumetricBlockInfo(const spatial_hash::Block& block, uint64_t update_stamp_ns = 0);
  VolumetricBlockInfo(const spatial_hash::BlockIndex& index,
                      double block_size,
                      uint64_t update_stamp_ns = 0);
  /// @brief World-frame center of the block.
  Eigen::Vector3f blockCenter() const;
};

/// @brief Interface for an active-window policy over the volumetric map.
struct VolumetricWindow {
  virtual ~VolumetricWindow() = default;

  /// @brief Archive every map block that is out of bounds for this pose.
  /// @param timestamp_ns Current timestamp (ns).
  /// @param world_T_body Current body pose in the world frame.
  /// @param map Map to archive blocks from (modified in place).
  /// @param skip_updated Skip blocks updated on this pass.
  /// @return Indices of the blocks that were archived.
  spatial_hash::BlockIndices archiveBlocks(uint64_t timestamp_ns,
                                           const Eigen::Isometry3d& world_T_body,
                                           VolumetricMap& map,
                                           bool skip_updated = true) const;

  /// @brief Whether the block lies within the active window for this pose.
  virtual bool inBounds(uint64_t timestamp_ns,
                        const Eigen::Isometry3d& world_T_body,
                        const VolumetricBlockInfo& block) const = 0;
};

/// @brief Window policy keeping blocks within a fixed radius of the body.
struct SpatialWindowChecker : VolumetricWindow {
  struct Config {
    double max_radius_m = 8.0;  ///< Radius of the active window around the body (m).
  } const config;

  explicit SpatialWindowChecker(const Config& config);
  bool inBounds(uint64_t timestamp_ns,
                const Eigen::Isometry3d& world_T_body,
                const VolumetricBlockInfo& block) const override;
};

void declare_config(SpatialWindowChecker::Config& config);

}  // namespace hydra

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
#include <bitset>
#include <iostream>

#include "hydra/places/gvd_voxel.h"

/**
 * @file voxel_templates.h
 * @brief Local 3x3x3 neighborhood templates used to detect corner (vertex)
 *        configurations of the GVD. Encodes a neighborhood as a 27-bit mask,
 *        provides conversions between row-major and spatial-hash iteration
 *        order, and matches masks against per-axis corner patterns.
 */

namespace hydra::places {

/// @brief Builds and manipulates the 27-bit occupancy mask of a voxel's 3x3x3 neighborhood.
struct CubeFlagExtractor {
  using IndexRotation = Eigen::Matrix<int, 3, 3>;  ///< Integer permutation matrix used to rotate masks.

  /// @brief Convert a neighborhood mask from row-major order to spatial-hash offset order.
  static std::bitset<27> fromRowMajor(const std::bitset<27>& flags_row_major);

  /// @brief Convert a neighborhood mask from spatial-hash offset order to row-major order.
  static std::bitset<27> toRowMajor(const std::bitset<27>& flags_sh);

  /// @brief Extract the neighborhood mask around a voxel in spatial-hash offset order.
  /// @param layer GVD layer to sample.
  /// @param index Center voxel index.
  /// @param min_extra_basis Minimum basis-point count for a neighbor bit to be set.
  static std::bitset<27> extract(const GvdLayer& layer,
                                 const GlobalIndex& index,
                                 uint8_t min_extra_basis = 1);

  /// @brief Rotate a spatial-hash-order neighborhood mask by a permutation matrix.
  static std::bitset<27> rotate(const IndexRotation& rotation,
                                const std::bitset<27>& flags_sh);

  const static std::vector<GlobalIndex> sh_offsets;                       ///< Neighbor offsets in spatial-hash iteration order.
  const static spatial_hash::IndexHashMap<size_t> sh_offset_lookup;       ///< Lookup from offset to its index in sh_offsets.
};

/// @brief A foreground/ignore mask pair describing one corner configuration.
struct GvdCornerTemplate {
  using MaskArray = std::array<std::bitset<27>, 4>;  ///< The four rotational variants of the don't-care mask.

  GvdCornerTemplate();

  /// @brief Build a template from row-major foreground and don't-care masks.
  GvdCornerTemplate(std::bitset<27> fg_mask_row_major,
                    MaskArray unused_mask_array_row_major);

  /// @brief Return whether a neighborhood state matches this template.
  bool matches(std::bitset<27> state) const;

  std::bitset<27> fg_mask;        ///< Bits that must be set for a match.
  MaskArray unused_mask_array;    ///< Per-rotation masks of bits ignored during matching.
};

/// @brief Stream a human-readable view of a corner template.
std::ostream& operator<<(std::ostream& out, const GvdCornerTemplate& corner_template);

/// @brief Holds the six per-axis corner templates and tests a neighborhood against all of them.
struct CornerFinder {
  CornerFinder();

  ~CornerFinder() = default;

  /// @brief Return whether the neighborhood mask matches any corner template.
  bool match(std::bitset<27> values) const;

  GvdCornerTemplate negative_x_template;  ///< Corner pattern oriented toward -x.
  GvdCornerTemplate positive_x_template;  ///< Corner pattern oriented toward +x.
  GvdCornerTemplate negative_y_template;  ///< Corner pattern oriented toward -y.
  GvdCornerTemplate positive_y_template;  ///< Corner pattern oriented toward +y.
  GvdCornerTemplate negative_z_template;  ///< Corner pattern oriented toward -z.
  GvdCornerTemplate positive_z_template;  ///< Corner pattern oriented toward +z.
};

}  // namespace hydra::places

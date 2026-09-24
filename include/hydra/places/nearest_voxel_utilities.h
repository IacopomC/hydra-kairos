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
#include <memory>
#include <unordered_set>
#include <vector>

#include "hydra/reconstruction/voxel_types.h"

/**
 * @file nearest_voxel_utilities.h
 * @brief Spatial queries over a set of voxel indices: finding the nearest
 *        indices to a query point and locating the index furthest from a line
 *        segment. Used when proposing and validating edges between places.
 */

namespace hydra::places {

/// @brief Answers nearest-neighbor queries over a fixed set of voxel indices.
class NearestVoxelFinder {
 public:
  /// @brief Invoked per result with the neighbor index, its rank, and its squared distance.
  using Callback = std::function<void(const GlobalIndex&, size_t, int64_t)>;

  /// @brief Build a finder over the given set of indices.
  explicit NearestVoxelFinder(const GlobalIndices& indices);

  virtual ~NearestVoxelFinder();

  /// @brief Report the nearest indices to a query, invoking the callback for each.
  /// @param index Query index.
  /// @param num_to_find Maximum number of neighbors to report.
  /// @param callback Receives each neighbor in order of increasing distance.
  void find(const GlobalIndex& index, size_t num_to_find, const Callback& callback);

 private:
  struct Detail;  ///< Opaque implementation holder.

  std::unique_ptr<Detail> internals_;  ///< Pointer to the implementation detail.
};

/// @brief Outcome of searching for the index furthest from a line segment.
struct FurthestIndexResult {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  bool valid = false;       ///< Whether a furthest index was found.
  int64_t distance = 0;     ///< Squared distance from the index to the line.
  bool from_source = true;  ///< Whether the index belongs to the source partition of the input.
  GlobalIndex index;        ///< The furthest index.
};

/// @brief Find the index in a set that lies furthest from the segment start-end.
/// @param indices Candidate indices, with the first number_source_edges treated as the source partition.
/// @param start Segment start index.
/// @param end Segment end index.
/// @param number_source_edges Size of the leading source partition.
FurthestIndexResult findFurthestIndexFromLine(
    const GlobalIndices& indices,
    const GlobalIndex& start,
    const GlobalIndex& end,
    size_t number_source_edges);

/// @brief Overload treating the entire index set as the source partition.
inline FurthestIndexResult findFurthestIndexFromLine(
    const GlobalIndices& indices,
    const GlobalIndex& start,
    const GlobalIndex& end) {
  return findFurthestIndexFromLine(indices, start, end, indices.size());
}

}  // namespace hydra::places

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
#include "hydra/places/gvd_integrator_config.h"
#include "hydra/places/gvd_voxel.h"
#include "hydra/reconstruction/voxel_types.h"

/**
 * @file gvd_utilities.h
 * @brief Free functions used during GVD integration: choosing the lower of two
 *        candidate distances and deciding whether a voxel pair meets the
 *        Voronoi condition based on parent-voxel separation.
 */

namespace hydra::places {

/// @brief Result of comparing a candidate distance against a current one.
struct DistancePotential {
  bool is_lower;    ///< Whether the candidate is the lower (closer) distance.
  double distance;  ///< The selected distance value.
};

/// @brief Per-voxel outcome of a Voronoi membership check on a voxel pair.
struct VoronoiCondition {
  bool neighbor_is_voronoi{false};  ///< Whether the neighbor should be marked Voronoi.
  bool current_is_voronoi{false};   ///< Whether the current voxel should be marked Voronoi.
};

/// @brief Pick the lower of a propagated neighbor distance and the current distance.
/// @param v_dist Current voxel distance.
/// @param n_dist Neighbor voxel distance.
/// @param distance Voxel-to-neighbor separation added during propagation.
/// @param min_diff_m Minimum improvement required to count as lower.
/// @return The chosen distance and whether it came from the neighbor.
DistancePotential getLowerDistance(float v_dist,
                                   float n_dist,
                                   float distance,
                                   float min_diff_m = 0.0f);

/// @brief Test whether a voxel and its neighbor form part of the Voronoi diagram
/// by checking that their nearest-surface parents are sufficiently separated.
VoronoiCondition checkVoronoi(const VoronoiCheckConfig& config,
                              const GvdVoxel& current,
                              const GlobalIndex& current_idx,
                              const GvdVoxel& neighbor,
                              const GlobalIndex& neighbor_idx);

/// @brief Parent-uniqueness test based on L1 distance between parent indices.
bool isParentUniqueL1(const VoronoiCheckConfig& config,
                      const GlobalIndex& /* current_index */,
                      const GlobalIndex& current_parent,
                      const GlobalIndex& neighbor_parent);

/// @brief Parent-uniqueness test based on the angle between parent directions.
bool isParentUniqueAngle(const VoronoiCheckConfig& config,
                         const GlobalIndex& current_index,
                         const GlobalIndex& current_parent,
                         const GlobalIndex& neighbor_parent);

/// @brief Parent-uniqueness test dispatching on the configured mode (L1, angle,
/// or L1 followed by angle).
bool isParentUnique(const VoronoiCheckConfig& config,
                    const GlobalIndex& current_idx,
                    const GlobalIndex& current_parent,
                    const GlobalIndex& neighbor_parent);

}  // namespace hydra::places

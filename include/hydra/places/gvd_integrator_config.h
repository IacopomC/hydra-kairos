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
#include <cstdint>

/**
 * @file gvd_integrator_config.h
 * @brief Configuration for the GVD/ESDF integrator and for the Voronoi
 *        membership test, including how the distinctness of two voxels'
 *        nearest-surface parents is measured.
 */

namespace hydra::places {

/// @brief Criterion for deciding when two nearest-surface parents are distinct.
enum class ParentUniquenessMode {
  ANGLE,         ///< Compare the angle between the directions to the two parents.
  L1_DISTANCE,   ///< Compare the L1 distance between the two parent indices.
  L1_THEN_ANGLE, ///< Require L1 separation first, then fall back to the angle test.
};

/// @brief Thresholds governing whether a voxel pair qualifies as Voronoi.
struct VoronoiCheckConfig {
  ParentUniquenessMode mode = ParentUniquenessMode::L1_THEN_ANGLE;  ///< Parent-distinctness criterion to apply.
  double min_distance_m = 0.2;             ///< Minimum clearance for a voxel to be eligible (m).
  double parent_l1_separation = 3.0;       ///< Minimum L1 separation between parents (voxels).
  double parent_cos_angle_separation = 0.5;  ///< Maximum cosine similarity between parent directions to count as distinct.
};

/// @brief Parameters controlling distance-field propagation and GVD extraction.
struct GvdIntegratorConfig {
  float max_distance_m = 2.0f;          ///< Largest distance tracked away from a surface (m).
  float min_distance_m = 0.2f;          ///< Smallest clearance considered free space (m).
  float min_diff_m = 1.0e-3f;           ///< Minimum distance change required to update a voxel (m).
  float min_weight = 1.0e-6f;           ///< Minimum TSDF weight for a voxel to be treated as observed.
  bool refine_voxel_pos = false;        ///< Whether to refine parent positions using the TSDF gradient.
  bool positive_distance_only = true;   ///< Whether to skip propagation into negative (inside-surface) distances.
  VoronoiCheckConfig voronoi_config;    ///< Settings for the Voronoi membership test.
};

/// @brief Bind VoronoiCheckConfig fields for serialization.
void declare_config(VoronoiCheckConfig& conf);

/// @brief Bind GvdIntegratorConfig fields for serialization.
void declare_config(GvdIntegratorConfig& conf);

}  // namespace hydra::places

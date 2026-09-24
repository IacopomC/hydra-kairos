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
#include <string>

/**
 * @file room_finder_config.h
 * @brief Configuration for the room finder: which clustering algorithm to use, how
 *        to choose the dilation threshold from the filtration curve, the size limits
 *        for components and rooms, and logging options.
 */

namespace hydra {

/// @brief Strategy used to group the selected place components into rooms.
enum class RoomClusterMode {
  MODULARITY,           ///< Greedy modularity using stored edge weights.
  MODULARITY_DISTANCE,  ///< Greedy modularity using inverse spatial distance as weight.
  NEIGHBORS,            ///< Weight-ordered flood fill from the seed components.
  NONE                  ///< Keep the seed components as-is, with no further clustering.
};

/// @brief Strategy used to pick the dilation threshold from the filtration curve.
enum class DilationThresholdMode {
  REPEATED,           ///< Longest run of a constant nonzero component count.
  LONGEST_LIFETIME,   ///< Dilation where qualifying components live the longest in total.
  PLATEAU,            ///< Largest plateau whose span exceeds a fraction of the longest.
  PLATEAU_THRESHOLD   ///< Largest plateau whose span exceeds an absolute threshold.
};

/// @brief Parameters controlling room detection from a places layer.
struct RoomFinderConfig {
  char room_prefix = 'R';                ///< Symbol prefix for generated room node IDs.
  double min_dilation_m = 0.1;           ///< Lower bound of the dilation search window (m).
  double max_dilation_m = 0.7;           ///< Upper bound of the dilation search window (m).
  double min_window_size = 0.2;          ///< Minimum span of the clipped window before it is discarded (m).
  bool clip_dilation_window_to_max = false;  ///< End the window at the component-count peak when set.
  size_t min_component_size = 15;        ///< Smallest component counted during filtration and kept as a seed.
  size_t min_room_size = 10;             ///< Smallest cluster promoted to a room node.
  DilationThresholdMode dilation_threshold_mode = DilationThresholdMode::REPEATED;  ///< How the threshold is chosen.
  double min_lifetime_length_m = 0.1;    ///< Minimum component lifetime, also the absolute plateau threshold (m).
  double plateau_ratio = 0.5;            ///< Fraction of the longest plateau span used by PLATEAU mode.
  RoomClusterMode clustering_mode = RoomClusterMode::NEIGHBORS;  ///< Algorithm grouping components into rooms.
  double max_modularity_iters = 5;       ///< Maximum passes for the modularity optimizer.
  double modularity_gamma = 1.0;         ///< Resolution parameter for the modularity optimizer.
  double dilation_diff_threshold_m = 1.0e-4;  ///< Distances closer than this collapse into one filtration sample (m).
  bool log_filtrations = false;          ///< Write the per-call filtration curve to a log file.
  bool log_place_graphs = false;         ///< Also serialize the place graphs alongside the filtration log.
};

/// @brief Register the config fields and enums for parsing and validation.
/// @param config Config instance whose schema is declared.
void declare_config(RoomFinderConfig& config);

}  // namespace hydra

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
#include <iostream>

/**
 * @file update_statistics.h
 * @brief Per-update counters summarizing the work the GVD/ESDF integrator
 *        performed in a single pass, useful for diagnostics and logging.
 */

namespace hydra::places {

/// @brief Counters describing what the integrator did during one update.
struct UpdateStatistics {
  size_t number_surface_flipped;   ///< Voxels whose sign (inside/outside) changed.
  size_t number_queue_inserts;     ///< Voxels pushed onto the propagation queue.
  size_t number_lowered_voxels;    ///< Voxels whose distance was decreased.
  size_t number_raised_voxels;     ///< Voxels whose distance was increased.
  size_t number_new_voxels;        ///< Newly observed voxels added.
  size_t number_raise_updates;     ///< Raise-wavefront update operations performed.
  size_t number_voronoi_found;     ///< Voxels newly marked as Voronoi members.
  size_t number_lower_skipped;     ///< Lower candidates rejected without an update.
  size_t number_lower_updated;     ///< Lower candidates that produced an update.
  size_t number_fixed_no_parent;   ///< Fixed-band voxels left without a parent.
  size_t number_force_lowered;     ///< Voxels lowered through a forced update.

  /// @brief Reset all counters to zero.
  void clear();
};

/// @brief Stream a human-readable summary of the counters.
std::ostream& operator<<(std::ostream& out, const UpdateStatistics& stats);

}  // namespace hydra::places

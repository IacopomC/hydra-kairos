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
/**
 * @file registration_solution.h
 * @brief Result of a loop-closure registration: the relative pose between two
 *        agent frames along with the hierarchy level and validity that the
 *        solver assigned to it.
 */
#pragma once
#include <Eigen/Geometry>
#include <cstdint>

#include "hydra/common/dsg_types.h"

namespace hydra::lcd {

/// @brief Relative-pose measurement linking a "from" agent frame to a "to"
/// agent frame, produced by a registration solver and consumed by the backend.
struct RegistrationSolution {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  bool valid = false;                            ///< Whether the solution holds a usable measurement.
  NodeId from_node;                              ///< Agent node defining the "from" frame.
  NodeId to_node;                                ///< Agent node defining the "to" frame.
  Eigen::Vector3d to_p_from;                     ///< Translation component of the to_T_from transform.
  Eigen::Quaterniond to_R_from;                  ///< Rotation component of the to_T_from transform.
  int64_t level;                                 ///< Hierarchy level the match was found at.
  std::optional<double> variance = std::nullopt; ///< Optional measurement variance.

  /// @brief Compose the stored translation and rotation into a transform.
  /// @return The to_T_from isometry mapping the "from" frame into the "to" frame.
  inline Eigen::Isometry3d to_T_from() const {
    return Eigen::Translation3d(to_p_from) * to_R_from;
  }
};

}  // namespace hydra::lcd

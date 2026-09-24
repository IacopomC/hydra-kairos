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
#include <config_utilities/factory.h>

#include "hydra/places/gvd_graph.h"

/**
 * @file gvd_merge_policies.h
 * @brief Pluggable strategies for deciding which of two GVD nodes is the better
 *        representative when nodes are merged during graph compression. Policies
 *        are selectable by name through the configuration factory.
 */

namespace hydra::places {

/// @brief Interface for ranking two GVD nodes against each other.
struct MergePolicy {
  virtual ~MergePolicy() = default;

  /// @brief Compare two GVD nodes to determine which is the better representative.
  /// @return 1 if lhs is better, -1 if rhs is better, 0 on a tie.
  virtual int compare(const GvdMemberInfo& lhs, const GvdMemberInfo& rhs) const = 0;
};

/// @brief Merge policy preferring the node with more supporting basis points.
struct BasisPointMergePolicy : MergePolicy {
  virtual ~BasisPointMergePolicy() = default;

  int compare(const GvdMemberInfo& lhs, const GvdMemberInfo& rhs) const override;

  inline static const auto registration_ =
      config::Registration<MergePolicy, BasisPointMergePolicy>("basis_points");  ///< Factory registration under "basis_points".
};

/// @brief Merge policy preferring the node with the larger clearance distance.
struct DistanceMergePolicy : MergePolicy {
  virtual ~DistanceMergePolicy() = default;

  int compare(const GvdMemberInfo& lhs, const GvdMemberInfo& rhs) const override;

  inline static const auto registration_ =
      config::Registration<MergePolicy, DistanceMergePolicy>("distance");  ///< Factory registration under "distance".
};

}  // namespace hydra::places

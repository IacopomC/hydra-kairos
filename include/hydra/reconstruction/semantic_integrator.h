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

/**
 * @file semantic_integrator.h
 * @brief Per-voxel semantic label fusion. Defines the SemanticIntegrator
 *        interface (which labels may be integrated and how a voxel's label
 *        likelihoods are updated) and a maximum-likelihood-estimate
 *        implementation.
 */

#include <set>
#include <cstdint>

#include "hydra/reconstruction/voxel_types.h"

namespace hydra {

/// @brief Interface for deciding which semantic labels enter the map and how a
/// voxel's label distribution is updated from an observed label.
struct SemanticIntegrator {
  virtual ~SemanticIntegrator() = default;

  /// @brief Whether a point with this label should be integrated at all.
  /// Discards points that should not be in the map, e.g. dynamic points or
  /// labels like 'sky'.
  virtual bool canIntegrate(uint32_t label) const = 0;

  /// @brief Whether this is a label whose semantics can be tracked. Distinct
  /// from canIntegrate: a label may be trackable even if the point itself is
  /// not integrated into the TSDF.
  virtual bool isValidLabel(uint32_t label) const = 0;

  /// @brief Fold an observed @p label into the voxel's label likelihoods.
  virtual void updateLikelihoods(uint32_t label, SemanticVoxel& voxel) const = 0;
};

/// @brief Maximum-likelihood semantic integrator, based in part on
/// Kimera-Semantics: maintains per-voxel label log-likelihoods under a
/// confusion model and reports the MLE label.
class MLESemanticIntegrator : public SemanticIntegrator {
 public:
  struct Config {
    double label_confidence = 0.9;  ///< Probability the observed label is correct.
  } const config;

  explicit MLESemanticIntegrator(const Config& config);

  bool canIntegrate(uint32_t label) const override;

  bool isValidLabel(uint32_t label) const override;

  void updateLikelihoods(uint32_t label, SemanticVoxel& voxel) const override;

 protected:
  size_t total_labels_;
  std::set<uint32_t> dynamic_labels_;
  std::set<uint32_t> invalid_labels_;

  float init_likelihood_;
  Eigen::MatrixXf observation_likelihoods_;
};

void declare_config(MLESemanticIntegrator::Config& config);

}  // namespace hydra

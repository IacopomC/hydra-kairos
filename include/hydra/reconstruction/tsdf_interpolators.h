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
 * @file tsdf_interpolators.h
 * @brief Resampling of a TSDF layer onto a coarser grid. Defines the
 *        TsdfInterpolator interface and a downsampling implementation, selected
 *        at runtime through the config factory.
 */

#include <config_utilities/factory.h>

#include <memory>

#include "hydra/reconstruction/voxel_types.h"

namespace hydra {

/// @brief Interface for resampling a TSDF layer into a new layer.
struct TsdfInterpolator {
  virtual ~TsdfInterpolator() = default;

  /// @brief Resample @p input, optionally restricted to @p blocks.
  /// @param input Source TSDF layer.
  /// @param blocks Block indices to limit the operation to, or null for all.
  /// @return The resampled layer.
  virtual std::shared_ptr<TsdfLayer> interpolate(const TsdfLayer& input,
                                                 const BlockIndices* blocks) const = 0;

  /// @brief Resample the whole @p input layer.
  std::shared_ptr<TsdfLayer> interpolate(const TsdfLayer& input) const {
    return interpolate(input, nullptr);
  }
};

/// @brief Interpolator that downsamples the TSDF by an integer block ratio.
struct DownsampleTsdfInterpolator : public TsdfInterpolator {
  struct Config {
    size_t ratio = 2;          ///< Downsampling factor along each axis.
    float tolerance = 1.0e-10; ///< Numerical tolerance for combining voxels.
  } const config;

  explicit DownsampleTsdfInterpolator(const Config& config);

  virtual ~DownsampleTsdfInterpolator() = default;

  std::shared_ptr<TsdfLayer> interpolate(const TsdfLayer& input,
                                         const BlockIndices* blocks) const override;

  inline static const auto registration_ =
      config::RegistrationWithConfig<TsdfInterpolator,
                                     DownsampleTsdfInterpolator,
                                     DownsampleTsdfInterpolator::Config>("downsample");
};

void declare_config(DownsampleTsdfInterpolator::Config& config);

}  // namespace hydra

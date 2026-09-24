// The contents of this file are originally from Panoptic-Mapping,
// under the following license:
//
// BSD 3-Clause License
// Copyright (c) 2021, ETHZ ASL
// All rights reserved.
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this
// list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// See https://github.com/ethz-asl/panoptic_mapping for original code and paper
//
// Modifications (including work done by Lukas Schmid for Khronos) fall under the same
// license as Hydra and are subject to the following copyright and disclaimer:
//
// Copyright 2022 Massachusetts Institute of Technology.
// All Rights Reserved
//
// Research was sponsored by the United States Air Force Research Laboratory and
// the United States Air Force Artificial Intelligence Accelerator and was
// accomplished under Cooperative Agreement Number FA8750-19-2-1000. The views
// and conclusions contained in this document are those of the authors and should
// not be interpreted as representing the official policies, either expressed or
// implied, of the United States Air Force or the U.S. Government. The U.S.
// Government is authorized to reproduce and distribute reprints for Government
// purposes notwithstanding any copyright notation herein.
#pragma once

/**
 * @file projection_interpolators.h
 * @brief Strategies for sampling sensor images at a projected sub-pixel point.
 *        Defines the ProjectionInterpolator interface and its nearest-neighbour,
 *        bilinear, and adaptive implementations, used when projecting a voxel
 *        into the range/color/label images during integration.
 */

#include <spark_dsg/color.h>

#include <memory>
#include <opencv2/core/mat.hpp>
#include <string>

namespace hydra {

/// @brief Cached interpolation state for one projected pixel: validity, the
/// integer pixel coordinates, the four neighbour weights, and whether bilinear
/// blending (rather than nearest) is in use.
struct InterpolationWeights {
  bool valid = false;
  int u;
  int v;
  float w0, w1, w2, w3;
  bool use_bilinear = false;

  InterpolationWeights() = default;
  InterpolationWeights(int _u, int _v) : u(_u), v(_v) {}
};

/// @brief Interface for sampling image values at a projected point. Call
/// computeWeights() first, then the interpolate* methods to read each image.
class ProjectionInterpolator {
 public:
  virtual ~ProjectionInterpolator() = default;

  /// @brief Compute the neighbour weights for a query point. Assumes (u, v) lie
  /// within the range image bounds, which is not re-checked here.
  /// @param u Horizontal image-space position of the point to interpolate.
  /// @param v Vertical image-space position of the point to interpolate.
  /// @param range_image Range image (32FC1) used to compute the weights.
  /// @return The weights used to interpolate quantities from the other images.
  virtual InterpolationWeights computeWeights(float u,
                                              float v,
                                              const cv::Mat& range_image) const = 0;

  /// @brief Interpolate the range/depth value using the given weights.
  /// @param range_image Range image (32FC1) to sample.
  /// @return The interpolated range value.
  virtual float InterpolateRange(const cv::Mat& range_image,
                                 const InterpolationWeights& weights) const = 0;

  /// @brief Interpolate the color using the given weights.
  /// @param color_img Color image (RGB8) to sample.
  /// @param w Interpolation weights.
  /// @return The interpolated color.
  virtual spark_dsg::Color interpolateColor(const cv::Mat& color_img,
                                            const InterpolationWeights& w) const = 0;

  /// @brief Interpolate the semantic id using the given weights.
  /// @param id_image ID image (8UC1) to sample.
  /// @return The interpolated ID.
  virtual int interpolateID(const cv::Mat& id_image,
                            const InterpolationWeights& weights) const = 0;
};

/// @brief Interpolator that selects the nearest neighbouring pixel.
class InterpolatorNearest : public ProjectionInterpolator {
 public:
  struct Config {};
  explicit InterpolatorNearest(const Config&) {}

  InterpolationWeights computeWeights(float u,
                                      float v,
                                      const cv::Mat& range_image) const override;

  float InterpolateRange(const cv::Mat& range_image,
                         const InterpolationWeights& weights) const override;

  spark_dsg::Color interpolateColor(const cv::Mat& color_image,
                                    const InterpolationWeights& weights) const override;

  int interpolateID(const cv::Mat& id_image,
                    const InterpolationWeights& weights) const override;
};

void declare_config(InterpolatorNearest::Config& config);

/// @brief Interpolator that blends the four neighbouring pixels bilinearly.
class InterpolatorBilinear : public ProjectionInterpolator {
 public:
  struct Config {};
  explicit InterpolatorBilinear(const Config&) {}

  InterpolationWeights computeWeights(float u,
                                      float v,
                                      const cv::Mat& range_image) const override;

  float InterpolateRange(const cv::Mat& range_image,
                         const InterpolationWeights& weights) const override;

  spark_dsg::Color interpolateColor(const cv::Mat& color_image,
                                    const InterpolationWeights& weights) const override;

  int interpolateID(const cv::Mat& id_image,
                    const InterpolationWeights& weights) const override;
};

void declare_config(InterpolatorBilinear::Config& config);

/// @brief Interpolator that uses bilinear blending when the neighbouring range
/// values are close, but falls back to nearest-neighbour at depth
/// discontinuities so surface edges are not smeared across the gap.
class InterpolatorAdaptive : public InterpolatorBilinear {
 public:
  struct Config {
    float max_depth_difference_m = 0.2;  ///< Range gap above which it falls back to nearest.
  } const config;

  explicit InterpolatorAdaptive(const Config& config);

  InterpolationWeights computeWeights(float u,
                                      float v,
                                      const cv::Mat& range_image) const override;

  float InterpolateRange(const cv::Mat& range_image,
                         const InterpolationWeights& weights) const override;

  spark_dsg::Color interpolateColor(const cv::Mat& color_image,
                                    const InterpolationWeights& weights) const override;

  int interpolateID(const cv::Mat& id_image,
                    const InterpolationWeights& weights) const override;

 private:
  const int u_offset_[4] = {0, 0, 1, 1};
  const int v_offset_[4] = {0, 1, 0, 1};
};

void declare_config(InterpolatorAdaptive::Config& config);

}  // namespace hydra

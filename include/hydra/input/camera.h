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
 * @file camera.h
 * @brief Pinhole-camera sensor model: projects points through the intrinsics,
 *        builds a vertex map and range image from a depth image, and tests the
 *        view frustum. Specializes the Sensor interface.
 */

#include <vector>

#include "hydra/input/input_data.h"
#include "hydra/input/sensor.h"

namespace hydra {

/// @brief Pinhole RGB-D camera sensor: intrinsics, point projection, and the
/// depth-to-vertex-map and vertex-map-to-range-image conversions.
class Camera : public Sensor {
 public:
  // Note: negative parameters are REQUIRED
  struct Config : public Sensor::Config {
    /// Camera resolution (columns)
    int width = -1;
    /// Camera resolution (rows)
    int height = -1;
    /// Camera center point (x-axis)
    float cx = -1.0;
    /// Camera center point (y-axis)
    float cy = -1.0;
    /// Camera focal length (x-axis)
    float fx = -1.0;
    /// Camera focal length (y-axis)
    float fy = -1.0;
  };

  explicit Camera(const Config& config, const std::string& name);

  virtual ~Camera() = default;

  /// @brief This camera's configuration.
  const Config& getConfig() const { return config_; }

  float getPointDepth(const Eigen::Vector3f& p) const override;

  float computeRayDensity(float voxel_size, float depth) const override;

  bool finalizeRepresentations(InputData& input,
                               bool force_world_frame = false) const override;

  bool projectPointToImagePlane(const Eigen::Vector3f& p_C,
                                float& u,
                                float& v) const override;

  bool projectPointToImagePlane(const Eigen::Vector3f& p_C,
                                int& u,
                                int& v) const override;

  bool pointIsInViewFrustum(const Eigen::Vector3f& point_C,
                            float inflation_distance = 0.0f) const override;

  /// @brief Back-project a depth image into a per-pixel vertex map, optionally
  /// transformed into the world frame via @p T_W_C.
  cv::Mat computeVertexMap(const cv::Mat& depth_image,
                           const Eigen::Isometry3f* T_W_C = nullptr) const;

  /// @brief Convert a vertex map into a range image, reporting its min/max range.
  cv::Mat computeRangeImage(const cv::Mat& vertex_map,
                            float* min_range,
                            float* max_range) const;

  YAML::Node dump() const override;

 private:
  const Config config_;

  Eigen::Matrix<float, 4, 3> view_frustum_;  ///< Top, right, bottom, left frustum plane normals.
};

void declare_config(Camera::Config& config);

}  // namespace hydra

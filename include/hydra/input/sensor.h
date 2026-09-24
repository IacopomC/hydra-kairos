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
 * @file sensor.h
 * @brief Sensor models for the reconstruction front end. Defines the Sensor base
 *        class (range bounds, body-to-sensor extrinsics, point projection into
 *        the image, view-frustum tests, and building the alternate range/cloud
 *        representations) that cameras and LiDARs specialize.
 */

#include <config_utilities/virtual_config.h>

#include <Eigen/Geometry>
#include <limits>
#include <vector>

#include "hydra/input/sensor_extrinsics.h"

namespace hydra {

struct InputData;

/// @brief Base class for the sensors the system can use. The design centers on
/// any sensor that can be expressed through images, such as RGB-D cameras or
/// LiDARs.
class Sensor {
 public:
  using Ptr = std::shared_ptr<Sensor>;
  using ConstPtr = std::shared_ptr<const Sensor>;

  struct Config {
    double min_range = 0.0f;                                  ///< Minimum valid range (m).
    double max_range = std::numeric_limits<double>::infinity();  ///< Maximum valid range (m).
    // TODO(nathan) try to avoid pulling in factories in the header
    config::VirtualConfig<SensorExtrinsics> extrinsics;      ///< Body-to-sensor extrinsics.
  } const config;

  explicit Sensor(const Config& config, const std::string& name);

  virtual ~Sensor() = default;

  /// @brief Minimum valid range of the sensor.
  virtual float min_range() const { return config.min_range; }

  /// @brief Maximum valid range of the sensor.
  virtual float max_range() const { return config.max_range; }

  /// @brief The body-to-sensor transform (the sensor pose in the body frame).
  template <typename T = double>
  Eigen::Transform<T, 3, Eigen::Isometry> body_T_sensor() const {
    Eigen::Isometry3d extrinsics_transform = *extrinsics_;
    return extrinsics_transform.cast<T>();
  }

  /// @brief Depth of a sensor-frame point, defined per sensor type.
  /// @param p A 3D point in the sensor frame.
  virtual float getPointDepth(const Eigen::Vector3f& p) const = 0;

  /// @brief Average ray density at a voxel of @p voxel_size at range @p depth.
  virtual float computeRayDensity(float voxel_size, float depth) const = 0;

  /// @brief Compute any alternate data representations the integrator needs.
  ///
  /// For cameras this builds a range image and point cloud from the depth; for
  /// LiDAR it builds a range image from the point cloud.
  ///
  /// @param input Input struct to fill.
  /// @param force_world_frame Always emit the point cloud in the world frame
  /// rather than the sensor frame.
  /// @return True if the sensor could provide all necessary information.
  virtual bool finalizeRepresentations(InputData& input,
                                       bool force_world_frame = false) const = 0;

  /// @brief Project a camera-frame (C) point onto the image plane.
  /// @param p_C Point in the camera frame.
  /// @param u Output x image coordinate (px).
  /// @param v Output y image coordinate (px).
  /// @return True if the point is in front of the camera and within the image.
  virtual bool projectPointToImagePlane(const Eigen::Vector3f& p_C,
                                        float& u,
                                        float& v) const = 0;

  /// @brief Project a camera-frame (C) point onto the image plane, rounded to
  /// the nearest integer pixel.
  /// @param p_C Point in the camera frame.
  /// @param u Output x image coordinate (px).
  /// @param v Output y image coordinate (px).
  /// @return True if the point is in front of the camera and within the image.
  virtual bool projectPointToImagePlane(const Eigen::Vector3f& p_C,
                                        int& u,
                                        int& v) const = 0;

  /// @brief Whether a point lies in the view frustum (no occlusion check).
  /// @param point_C Point in the camera frame.
  /// @param inflation_distance Distance to inflate the frustum by.
  /// @return True if the point is in the (inflated) view frustum.
  virtual bool pointIsInViewFrustum(const Eigen::Vector3f& point_C,
                                    float inflation_distance = 0.f) const = 0;

  const std::string name;  ///< Name of this sensor.

  /// @brief Serialize the sensor configuration to YAML.
  virtual YAML::Node dump() const;

 protected:
  const std::unique_ptr<SensorExtrinsics> extrinsics_;
};

void declare_config(Sensor::Config& config);

}  // namespace hydra

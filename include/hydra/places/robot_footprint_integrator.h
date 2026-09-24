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

#include <Eigen/Geometry>
#include <memory>

#include "hydra/common/dsg_types.h"

/**
 * @file robot_footprint_integrator.h
 * @brief Carves the robot's own footprint as free space into the volumetric
 *        map, so the body never appears as an obstacle to the distance field.
 */

namespace hydra {

class VolumetricMap;  ///< Forward declaration of the map being written into.

/// @brief Marks a body-frame bounding box as free space in the TSDF.
class RobotFootprintIntegrator {
 public:
  using Ptr = std::shared_ptr<RobotFootprintIntegrator>;  ///< Shared-pointer alias.

  /// @brief Footprint extent and the weight used when writing it.
  struct Config {
    Eigen::Vector3f bbox_min = Eigen::Vector3f::Zero();  ///< Lower corner of the footprint box in the body frame (m).
    Eigen::Vector3f bbox_max = Eigen::Vector3f::Zero();  ///< Upper corner of the footprint box in the body frame (m).
    double tsdf_weight = 1.0e-5;                         ///< Weight applied to voxels marked free.
  };

  /// @brief Construct from a footprint configuration.
  RobotFootprintIntegrator(const Config& config);

  virtual ~RobotFootprintIntegrator();

  /// @brief Mark the footprint as free space in the map at the given body pose.
  /// @param world_T_body Transform placing the body frame in the world.
  /// @param map Volumetric map updated in place.
  void markFreespace(const Eigen::Isometry3f& world_T_body, VolumetricMap& map) const;

 public:
  const Config config;    ///< Footprint configuration.
  const BoundingBox bbox; ///< Bounding box derived from the configured corners.

 private:
  inline static const auto registration_ =
      config::RegistrationWithConfig<RobotFootprintIntegrator,
                                     RobotFootprintIntegrator,
                                     Config>("RobotFootprintIntegrator");  ///< Factory registration under "RobotFootprintIntegrator".
};

/// @brief Bind RobotFootprintIntegrator::Config fields for serialization.
void declare_config(RobotFootprintIntegrator::Config& config);

}  // namespace hydra

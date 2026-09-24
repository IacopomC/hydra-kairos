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
#include <config_utilities/virtual_config.h>

#include <Eigen/Geometry>

#include "hydra/active_window/active_window_module.h"
#include "hydra/reconstruction/mesh_integrator_config.h"
#include "hydra/reconstruction/projective_integrator.h"

/**
 * @file reconstruction_module.h
 * @brief Active-window subclass that reconstructs the volumetric map: it fuses
 *        each frame's depth/semantics into the TSDF, meshes the dirty blocks,
 *        and integrates the robot footprint into free space.
 */

namespace hydra {

class ProjectiveIntegrator;
class MeshIntegrator;
class RobotFootprintIntegrator;

/// @brief Active-window module that runs TSDF, mesh, and footprint integration per frame.
class ReconstructionModule : public ActiveWindowModule {
 public:
  struct Config : ActiveWindowModule::Config {
    double full_update_separation_s = 0.0;  ///< Minimum time between full map updates (s); 0 updates every frame.
    ProjectiveIntegrator::Config tsdf;  ///< TSDF projective-integrator configuration.
    MeshIntegratorConfig mesh;  ///< Mesh-integrator configuration.
    config::VirtualConfig<RobotFootprintIntegrator> robot_footprint;  ///< Optional robot-footprint integrator.
  } const config;

  ReconstructionModule(const Config& config, const OutputQueue::Ptr& output_queue);

  virtual ~ReconstructionModule();

  /// @brief Return a human-readable summary of module state.
  std::string printInfo() const override;

 protected:
  /// @brief Whether a full update is due at this timestamp per full_update_separation_s.
  bool shouldUpdate(uint64_t timestamp_ns) const;

  /// @brief Integrate one input frame into the map and emit the active-window output.
  ActiveWindowOutput::Ptr spinOnce(const InputPacket& input) override;

 protected:
  std::optional<uint64_t> last_update_ns_;  ///< Timestamp of the last full update.

  std::unique_ptr<ProjectiveIntegrator> tsdf_integrator_;  ///< Fuses depth/semantics into the TSDF.
  std::unique_ptr<MeshIntegrator> mesh_integrator_;  ///< Builds the surface mesh from the TSDF.
  std::unique_ptr<RobotFootprintIntegrator> footprint_integrator_;  ///< Carves the robot footprint into free space.
};

void declare_config(ReconstructionModule::Config& config);

}  // namespace hydra

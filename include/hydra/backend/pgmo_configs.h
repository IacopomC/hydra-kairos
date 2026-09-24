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
#include <KimeraRPGO/SolverParams.h>
#include <kimera_pgmo/kimera_pgmo_interface.h>

/**
 * @file pgmo_configs.h
 * @brief Extends the pose-graph-and-mesh-optimization configuration with the
 *        scene-graph factor covariances and robust-solver settings used by the
 *        backend optimizer.
 */

namespace hydra {

/// @brief Pose-graph-and-mesh-optimization config plus scene-graph factor
/// covariances and robust-solver settings.
struct HydraPgmoConfig : public kimera_pgmo::KimeraPgmoConfig {
  double place_mesh_variance;       ///< Variance on place-to-mesh deformation edges.
  double place_edge_variance;       ///< Variance on place-to-place deformation edges.
  double place_merge_variance;      ///< Variance on factors anchoring merged places.
  double object_merge_variance;     ///< Variance on factors anchoring merged objects.
  double sg_loop_closure_variance;  ///< Variance on scene-graph loop-closure factors.
  bool gnc_fix_prev_inliers = true;  ///< Keep previously accepted inliers fixed during robust optimization.
  KimeraRPGO::Verbosity rpgo_verbosity = KimeraRPGO::Verbosity::UPDATE;  ///< Robust-solver logging verbosity.
  KimeraRPGO::Solver rpgo_solver = KimeraRPGO::Solver::LM;  ///< Nonlinear solver backing the robust optimizer.
};

/// @brief Declares the configuration fields for HydraPgmoConfig.
void declare_config(HydraPgmoConfig& conf);

}  // namespace hydra

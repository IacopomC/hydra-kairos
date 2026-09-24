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
 * @file loop_closure_config.h
 * @brief Top-level configuration for the loop-closure module and the
 *        declarations that register every loop-closure config struct with the
 *        configuration framework.
 */
#pragma once
#include "hydra/loop_closure/bow_agent_solver.h"
#include "hydra/loop_closure/detector.h"

namespace hydra {

/// @brief Configuration for the loop-closure module, wrapping the detector
/// settings together with module-level visualization and horizon parameters.
struct LoopClosureConfig {
  lcd::LcdDetectorConfig detector;        ///< Detector configuration.
  bool visualize_dsg_lcd = false;          ///< Publish visualization output when true.
  std::string lcd_visualizer_ns = "/dsg/lcd_visualizer";  ///< Namespace for visualization topics.
  double lcd_agent_horizon_s = 1.5;        ///< Time window for selecting the query agent (s).
  double descriptor_creation_horizon_m = 10.0;  ///< Radius around the query agent within which places are cached for description (m).
  /// Agent-layer solver consulted when detector.enable_agent_registration is
  /// set and no other solver has been installed at the agent layer. Letting it
  /// resolve a match short-circuits the cascade before the place layer runs.
  lcd::BowAgentSolver::Config bow_agent;
};

namespace lcd {
/// @brief Register the agent solver configuration fields.
void declare_config(BowAgentSolver::Config& conf);
/// @brief Register a per-layer matching and registration configuration.
void declare_config(LayerLcdConfig& conf);
/// @brief Register the layer registration configuration fields.
void declare_config(LayerRegistrationConfig& conf);
/// @brief Register the descriptor match configuration fields.
void declare_config(DescriptorMatchConfig& conf);
/// @brief Register the graph-network descriptor configuration fields.
void declare_config(GnnLcdConfig& conf);
/// @brief Register the detector configuration fields.
void declare_config(LcdDetectorConfig& conf);
}  // namespace lcd

/// @brief Register the subgraph extraction configuration fields.
void declare_config(SubgraphConfig& conf);
/// @brief Register the loop-closure module configuration fields.
void declare_config(LoopClosureConfig& conf);

}  // namespace hydra

namespace teaser {
/// @brief Register the robust registration solver parameters.
void declare_config(RobustRegistrationSolver::Params& conf);
}  // namespace teaser

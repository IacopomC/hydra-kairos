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

#include "hydra/backend/update_functions.h"

/**
 * @file update_buildings_functor.h
 * @brief Update functor for the buildings layer. Maintains a single building
 *        node placed at the centroid of all rooms and parents every room to it;
 *        the building node is removed when no rooms remain.
 */

namespace hydra {

/// @brief Maintains the building node summarizing the rooms layer.
struct UpdateBuildingsFunctor : public UpdateFunctor {
  struct Config {
    SemanticNodeAttributes::Label semantic_label = 22u;  ///< Semantic label assigned to the building node.
  } const config;

  /// @brief Constructs the functor from a validated configuration.
  explicit UpdateBuildingsFunctor(const Config& config);

  /// @brief Places the building node at the room centroid and reparents the rooms.
  /// @param unmerged Graph holding the original, unmerged nodes.
  /// @param dsg Shared graph whose building node is maintained.
  /// @param info Per-iteration context.
  void call(const DynamicSceneGraph& unmerged,
            SharedDsgInfo& dsg,
            const UpdateInfo::ConstPtr& info) const override;

 private:
  inline static const auto registration_ =
      config::RegistrationWithConfig<UpdateFunctor, UpdateBuildingsFunctor, Config>(
          "UpdateBuildingsFunctor");
};

/// @brief Declares the configuration fields for UpdateBuildingsFunctor.
void declare_config(UpdateBuildingsFunctor::Config& config);

}  // namespace hydra

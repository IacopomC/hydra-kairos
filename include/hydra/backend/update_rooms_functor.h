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
#include "hydra/rooms/room_finder.h"

/**
 * @file update_rooms_functor.h
 * @brief Update functor for the rooms layer. Re-derives rooms from the current
 *        places layer via a room finder and replaces the existing rooms with
 *        the newly detected ones.
 */

namespace hydra {

/// @brief Re-detects rooms from the places layer and rewrites the rooms layer.
struct UpdateRoomsFunctor : public UpdateFunctor {
  struct Config {
    RoomFinderConfig room_finder;  ///< Configuration for the room-detection algorithm.
  } const config;

  /// @brief Constructs the functor and its room finder from the configuration.
  explicit UpdateRoomsFunctor(const Config& config);

  /// @brief Runs room detection over the places layer and rewrites the rooms.
  /// @param unmerged Graph holding the original, unmerged nodes.
  /// @param dsg Shared graph whose rooms layer is rewritten.
  /// @param info Per-iteration context.
  void call(const DynamicSceneGraph& unmerged,
            SharedDsgInfo& dsg,
            const UpdateInfo::ConstPtr& info) const override;

  /// @brief Replaces the graph's rooms with a freshly detected room layer.
  /// @param new_rooms Newly detected rooms, or null to clear the layer.
  /// @param graph Graph whose rooms layer is rewritten.
  void rewriteRooms(const SceneGraphLayer* new_rooms, DynamicSceneGraph& graph) const;

  std::unique_ptr<RoomFinder> room_finder;  ///< Algorithm that derives rooms from places.

 private:
  inline static const auto registration_ =
      config::RegistrationWithConfig<UpdateFunctor, UpdateRoomsFunctor, Config>(
          "UpdateRoomsFunctor");
};

/// @brief Declares the configuration fields for UpdateRoomsFunctor.
void declare_config(UpdateRoomsFunctor::Config& config);

}  // namespace hydra

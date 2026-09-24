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
#include <unordered_set>

#include "hydra/common/dsg_types.h"

/**
 * @file room_utilities.h
 * @brief Helpers for placing room nodes and wiring up room-to-room edges. Computes a
 *        representative position for a room from its member places and connects rooms
 *        whose places are adjacent in the places layer.
 */

namespace hydra {

/// @brief Compute a representative position for a room from its member places. Returns the
/// centroid when it lies within some place's free-space radius, otherwise projects the
/// centroid onto the nearest place's free-space boundary.
/// @param places Places layer holding the member node attributes.
/// @param cluster Member place IDs of the room.
/// @return The chosen room position, or the origin when the cluster is empty.
Eigen::Vector3d getRoomPosition(const SceneGraphLayer& places,
                                const std::unordered_set<NodeId>& cluster);

/// @brief Add an edge between two rooms whenever their member places are siblings in the
/// places layer.
/// @param places Places layer providing the adjacency between member places.
/// @param labels Cluster label assigned to each place.
/// @param label_to_room_map Maps each cluster label to its room node ID.
/// @param rooms Room layer that receives the inter-room edges.
void addEdgesToRoomLayer(const SceneGraphLayer& places,
                         const std::map<NodeId, size_t>& labels,
                         const std::map<size_t, NodeId> label_to_room_map,
                         SceneGraphLayer& rooms);

/// @brief Refresh the edges of the given rooms by clearing existing links among them and
/// reconnecting rooms whose member places are adjacent, descending through an archetype
/// layer when one is present.
/// @param graph Scene graph holding the rooms and the layers below them.
/// @param active_rooms Room node IDs whose edges are recomputed.
void addEdgesToRoomLayer(DynamicSceneGraph& graph,
                         const std::set<NodeId>& active_rooms);

}  // namespace hydra

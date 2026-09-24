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
#include "hydra/rooms/room_utilities.h"

namespace hydra {

Eigen::Vector3d getRoomPosition(const SceneGraphLayer& places,
                                const std::unordered_set<NodeId>& cluster) {
  if (cluster.empty()) {
    return Eigen::Vector3d::Zero();
  }

  // start from the plain centroid of the cluster's place positions
  Eigen::Vector3d room_position = Eigen::Vector3d::Zero();
  for (const auto& place : cluster) {
    room_position += getNodePosition(places, place);
  }
  room_position /= cluster.size();

  // the centroid can fall outside free space (e.g. in a wall for an L-shaped room).
  // each place carries a free-space radius (distance); check whether the centroid lands
  // inside any place's free-space ball, and meanwhile track the closest ball to it
  double best_radius = 0.0;
  Eigen::Vector3d best_position = Eigen::Vector3d::Zero();
  double best_distance = std::numeric_limits<double>::infinity();

  bool room_in_freespace = false;
  for (const auto& place : cluster) {
    const auto& attrs = places.getNode(place).attributes<PlaceNodeAttributes>();
    double room_distance = (room_position - attrs.position).norm();
    if (room_distance <= attrs.distance) {
      room_in_freespace = true;
      break;
    }

    double distance_to_freespace = room_distance - attrs.distance;
    if (distance_to_freespace < best_distance) {
      best_distance = distance_to_freespace;
      best_radius = attrs.distance;
      best_position = attrs.position;
    }
  }

  if (room_in_freespace) {
    return room_position;
  }

  // centroid was outside all free space: snap it onto the surface of the nearest
  // free-space ball so the room marker sits in navigable space
  Eigen::Vector3d unit_vector = (room_position - best_position).normalized();
  return best_position + unit_vector * best_radius;
}

void addEdgesToRoomLayer(const SceneGraphLayer& places,
                         const std::map<NodeId, size_t>& labels,
                         const std::map<size_t, NodeId> label_to_room_map,
                         SceneGraphLayer& rooms) {
  // connect two rooms whenever a place in one is adjacent to a place in the other:
  // a place-level edge crossing a room boundary becomes a room-level edge
  for (const auto& id_node_pair : places.nodes()) {
    const auto place = id_node_pair.first;
    const auto label = labels.find(place);
    if (label == labels.end()) {
      continue;  // place has no cluster
    }

    const auto parent = label_to_room_map.find(label->second);
    if (parent == label_to_room_map.end()) {
      continue;  // cluster has no room (e.g. below size threshold)
    }

    for (const auto& sibling : id_node_pair.second->siblings()) {
      const auto sibling_label = labels.find(sibling);
      if (sibling_label == labels.end()) {
        continue;
      }

      const auto sibling_parent = label_to_room_map.find(sibling_label->second);
      if (sibling_parent == label_to_room_map.end()) {
        continue;
      }

      if (parent->second == sibling_parent->second) {
        continue;  // both places in the same room: no inter-room edge
      }

      // repeated inserts don't matter
      rooms.insertEdge(parent->second, sibling_parent->second);
    }
  }
}

void addEdgesToRoomLayer(DynamicSceneGraph& graph,
                         const std::set<NodeId>& active_rooms) {
  // recompute inter-room edges among the active rooms directly on the full graph,
  // handling an optional intermediate archetype layer between rooms and places
  for (const auto node_id : active_rooms) {
    const auto& node = graph.getNode(node_id);
    // drop existing edges between two active rooms so they can be rebuilt from scratch;
    // edges to inactive rooms are left untouched
    const auto room_siblings = node.siblings();
    for (const auto other : room_siblings) {
      if (active_rooms.count(other)) {
        // clean potential edges
        graph.removeEdge(node_id, other);
      }
    }

    // Collect place-level children (may be direct or through archetype layer)
    std::vector<NodeId> place_children;
    for (const auto child_id : node.children()) {
      if (graph.isDynamic(child_id)) {
        continue;
      }
      const auto& child = graph.getNode(child_id);
      if (child.children().empty()) {
        // Direct child is a place (no archetype layer)
        place_children.push_back(child_id);
      } else {
        // Child is an archetype node — descend to its place children
        for (const auto grandchild_id : child.children()) {
          if (!graph.isDynamic(grandchild_id)) {
            place_children.push_back(grandchild_id);
          }
        }
      }
    }

    for (const auto place_id : place_children) {
      const auto& place = graph.getNode(place_id);
      for (const auto& sibling_id : place.siblings()) {
        // Walk up from sibling place to its room (through archetype if present)
        const auto& sibling = graph.getNode(sibling_id);
        auto parent_id = sibling.getParent();
        if (!parent_id) {
          continue;
        }
        // If parent is an archetype, go one more level up to find the room
        const auto& parent_node = graph.getNode(*parent_id);
        auto room_id = parent_node.getParent();
        if (room_id) {
          parent_id = room_id;
        }

        if (*parent_id == node_id) {
          continue;
        }

        graph.insertEdge(node_id, *parent_id);
      }
    }
  }
}

}  // namespace hydra

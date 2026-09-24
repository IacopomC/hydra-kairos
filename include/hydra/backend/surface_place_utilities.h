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
#include <glog/logging.h>

#include "hydra/common/dsg_types.h"
#include "hydra/frontend/place_2d_split_logic.h"

/**
 * @file surface_place_utilities.h
 * @brief Helpers that recompute the surface-place (2D place) layer after the
 *        mesh is optimized. They collect places flagged for finishing, refresh
 *        their geometry, split oversized places, and insert the resulting nodes
 *        and their connecting edges back into the graph.
 */

namespace hydra::utils {

/// @brief Collects surface places flagged for a finished merge and their neighbors.
/// @param places_layer Surface-places layer to scan.
/// @param place_2ds Output list of node ids paired with their reconstructed place geometry.
/// @param node_neighbors Output mapping from each collected node to its sibling nodes.
void getPlace2dAndNeighors(const SceneGraphLayer& places_layer,
                           std::vector<std::pair<NodeId, Place2d>>& place_2ds,
                           std::map<NodeId, std::set<NodeId>>& node_neighbors);

/// @brief Recomputes rectangle, boundary, and mesh-index bounds for each place.
/// @param mesh Mesh providing the vertex positions.
/// @param connection_ellipse_scale_factor Inflation factor applied to the place ellipse.
/// @param place_2ds Places whose geometry is recomputed.
/// @param nodes_to_update Output list of places ready to be written back unchanged in count.
void computeAttributeUpdates(const spark_dsg::Mesh& mesh,
                             const double connection_ellipse_scale_factor,
                             std::vector<std::pair<NodeId, Place2d>>& place_2ds,
                             std::vector<std::pair<NodeId, Place2d>>& nodes_to_update);

/// @brief Sorts places into those updated in place and those that must be split into new nodes.
/// @param mesh Mesh providing the vertex positions.
/// @param min_points Minimum vertex count before a place is eligible for splitting.
/// @param min_size Minimum spatial extent before a place is eligible for splitting.
/// @param connection_ellipse_scale_factor Inflation factor applied to the place ellipse.
/// @param place_2ds Candidate places to triage.
/// @param nodes_to_update Output places that keep their identity and are merely refreshed.
/// @param nodes_to_add Output original nodes paired with the set of places they decompose into.
void getNecessaryUpdates(
    const spark_dsg::Mesh& mesh,
    size_t min_points,
    double min_size,
    double connection_ellipse_scale_factor,
    std::vector<std::pair<NodeId, Place2d>>& place_2ds,
    std::vector<std::pair<NodeId, Place2d>>& nodes_to_update,
    std::vector<std::pair<NodeId, std::vector<Place2d>>>& nodes_to_add);

/// @brief Computes overlap weights between every pair of to-be-added split places.
/// @param nodes_to_add Original nodes paired with their decomposed places.
/// @param place_overlap_threshold Minimum overlap for two places to be connected.
/// @param place_neighbor_z_diff Maximum vertical separation for two places to be connected.
/// @return Map keyed by the two (parent, child) index pairs to their overlap weight.
std::map<std::tuple<size_t, size_t, size_t, size_t>, double> buildEdgeMap(
    const std::vector<std::pair<NodeId, std::vector<Place2d>>>& nodes_to_add,
    double place_overlap_threshold,
    double place_neighbor_z_diff);

/// @brief Writes refreshed geometry back onto the existing surface-place nodes.
/// @param nodes_to_update Nodes paired with their updated place geometry.
/// @param graph Graph whose nodes are updated.
void updateExistingNodes(const std::vector<std::pair<NodeId, Place2d>>& nodes_to_update,
                         DynamicSceneGraph& graph);

/// @brief Inserts the split places as new surface-place nodes.
/// @param nodes_to_add Original nodes paired with the places they decompose into.
/// @param place_overlap_threshold Minimum overlap for two places to be connected.
/// @param place_max_neighbor_z_diff Maximum vertical separation for two places to be connected.
/// @param next_node_symbol First node symbol to assign to the new nodes.
/// @param graph Graph the new nodes are inserted into.
/// @param new_id_map Output mapping from (parent, child) index pairs to the new node ids.
/// @return Next free node symbol after the inserted nodes.
NodeSymbol insertNewNodes(
    const std::vector<std::pair<NodeId, std::vector<Place2d>>>& nodes_to_add,
    const double place_overlap_threshold,
    const double place_max_neighbor_z_diff,
    NodeSymbol next_node_symbol,
    DynamicSceneGraph& graph,
    std::map<std::tuple<size_t, size_t>, NodeId>& new_id_map);

/// @brief Adds edges between newly inserted split places using precomputed weights.
/// @param nodes_to_add Original nodes paired with the places they decompose into.
/// @param edge_map Overlap weights keyed by the connected index pairs.
/// @param new_id_map Mapping from (parent, child) index pairs to the new node ids.
/// @param graph Graph the edges are added to.
void addNewNodeEdges(
    const std::vector<std::pair<NodeId, std::vector<Place2d>>> nodes_to_add,
    const std::map<std::tuple<size_t, size_t, size_t, size_t>, double> edge_map,
    const std::map<std::tuple<size_t, size_t>, NodeId> new_id_map,
    DynamicSceneGraph& graph);

/// @brief Redistributes shared mesh points between two surface places by proximity.
/// @param points Mesh vertices the connections index into.
/// @param attrs1 Attributes of the first place, updated in place.
/// @param attrs2 Attributes of the second place, updated in place.
void reallocateMeshPoints(const std::vector<Place2d::PointT>& points,
                          Place2dNodeAttributes& attrs1,
                          Place2dNodeAttributes& attrs2);

}  // namespace hydra::utils

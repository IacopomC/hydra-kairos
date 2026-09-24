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
#include <bitset>
#include <iostream>

#include "hydra/common/dsg_types.h"
#include "hydra/places/graph_extractor_config.h"
#include "hydra/places/gvd_voxel.h"
#include "hydra/reconstruction/voxel_types.h"

/**
 * @file graph_extractor_utilities.h
 * @brief Helpers shared by the place-graph extractors for proposing edges:
 *        rasterizing a voxel line between two indices, scoring candidate edges
 *        by their clearance, and gathering overlap-based and free-space edges.
 */

namespace hydra::places {

using EdgeInfoMap = std::map<EdgeKey, EdgeAttributes::Ptr>;     ///< Proposed edges keyed by endpoint pair.
using NodeIndexMap = std::unordered_map<NodeId, GlobalIndex>;   ///< Map from place node to its source voxel index.

/// @brief Rasterize the voxel indices along the straight line from start to end.
GlobalIndices makeBresenhamLine(const GlobalIndex& start, const GlobalIndex& end);

/// @brief Build edge attributes for two nodes whose free-space regions overlap.
/// @return Attributes if the edge clears min_edge_clearance_m, otherwise null.
EdgeAttributes::Ptr getOverlapEdgeInfo(const SceneGraphLayer& graph,
                                       NodeId node,
                                       NodeId neighbor,
                                       double min_edge_clearance_m);

/// @brief Build edge attributes for a free-space bridge between two nodes,
/// checking clearance along the voxel line connecting them.
/// @return Attributes if the path clears min_edge_clearance_m, otherwise null.
EdgeAttributes::Ptr getFreespaceEdgeInfo(const SceneGraphLayer& graph,
                                         const GvdLayer& gvd,
                                         const NodeIndexMap& node_index_map,
                                         NodeId node,
                                         NodeId other,
                                         double min_edge_clearance_m);

/// @brief Collect candidate edges between active nodes with overlapping free space.
/// @param proposed_edges Output map extended with the discovered edges.
void findOverlapEdges(const OverlapEdgeConfig& config,
                      const SceneGraphLayer& graph,
                      const std::unordered_set<NodeId> active_nodes,
                      EdgeInfoMap& proposed_edges);

/// @brief Collect candidate edges that bridge disconnected components through free space.
/// @param proposed_edges Output map extended with the discovered edges.
void findFreespaceEdges(const FreespaceEdgeConfig& config,
                        const SceneGraphLayer& graph,
                        const GvdLayer& gvd,
                        const std::unordered_set<NodeId>& nodes,
                        const NodeIndexMap& node_index_map,
                        EdgeInfoMap& proposed_edges);

}  // namespace hydra::places

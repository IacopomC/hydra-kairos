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
#include <kimera_pgmo/deformation_graph.h>

#include "hydra/common/dsg_types.h"

/**
 * @file mst_factors.h
 * @brief Adds the places layer to the deformation graph. Connects places along
 *        a minimum spanning tree of their connectivity and attaches each place
 *        to nearby mesh vertices, so the optimizer deforms the places together
 *        with the mesh.
 */

namespace hydra {

/// @brief Inserts places nodes and factors into the deformation graph.
/// @param graph Scene graph providing the places layer and their connectivity.
/// @param timestamp_ns Timestamp associated with the inserted factors (ns).
/// @param deformation_graph Deformation graph extended with the places and factors.
/// @param mst_edge_variance Measurement variance for the place-to-place tree edges.
/// @param mesh_edge_variance Measurement variance for the place-to-mesh edges.
/// @param prefix_lookup Maps a node id to the symbol prefix used for its deformation-graph key.
void addPlacesToDeformationGraph(const DynamicSceneGraph& graph,
                                 size_t timestamp_ns,
                                 kimera_pgmo::DeformationGraph& deformation_graph,
                                 double mst_edge_variance,
                                 double mesh_edge_variance,
                                 const std::function<char(NodeId)>& prefix_lookup);

}  // namespace hydra

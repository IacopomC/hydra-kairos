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
#include "hydra/loop_closure/registration.h"

#include <fstream>
#include <functional>
#include <iomanip>

#include "hydra/utils/timing_utilities.h"

namespace hydra::lcd {

inline std::ostream& operator<<(std::ostream& out, const gtsam::Quaternion& q) {
  out << "{w: " << q.w() << ", x: " << q.x() << ", y: " << q.y() << ", z: " << q.z()
      << "}";
  return out;
}

using hydra::timing::ScopedTimer;

struct AgentNodePose {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  bool valid = false;
  gtsam::Pose3 world_T_body;
  NodeId id;
};

AgentNodePose getAgentPose(const DynamicSceneGraph& graph, NodeId root_id) {
  if (!graph.hasNode(root_id)) {
    return {};
  }

  const auto& root_node = graph.getNode(root_id);

  // Recursively search for a dynamic agent child (may be through archetype layer)
  std::optional<NodeId> child_id = std::nullopt;
  std::function<bool(const SceneGraphNode&)> findAgent;
  findAgent = [&](const SceneGraphNode& node) -> bool {
    for (const auto& child : node.children()) {
      if (graph.isDynamic(child)) {
        child_id = child;
        return true;
      }
      if (findAgent(graph.getNode(child))) {
        return true;
      }
    }
    return false;
  };
  findAgent(root_node);

  if (!child_id) {
    return {};
  }

  CHECK(child_id);

  const auto& attrs = graph.getNode(*child_id).attributes<AgentNodeAttributes>();
  return {
      true, gtsam::Pose3(gtsam::Rot3(attrs.world_R_body), attrs.position), *child_id};
}

AgentNodePose getQueryPose(const DynamicSceneGraph& graph, NodeId query_agent_id) {
  const auto& attrs = graph.getNode(query_agent_id).attributes<AgentNodeAttributes>();
  return {true,
          gtsam::Pose3(gtsam::Rot3(attrs.world_R_body), attrs.position),
          query_agent_id};
}

std::string getPoseRepr(const gtsam::Pose3& pose,
                        int translation_precision = 4,
                        int rotation_precision = 4) {
  const Eigen::IOFormat format(
      translation_precision, Eigen::DontAlignCols, ", ", ", ", "", "", "[", "]");
  std::stringstream ss;
  ss << std::setprecision(rotation_precision) << std::fixed
     << "<R=" << pose.rotation().toQuaternion()
     << ", t=" << pose.translation().format(format) << ">";
  return ss.str();
}

RegistrationSolution getFullSolutionFromLayer(const DynamicSceneGraph& graph,
                                              const LayerRegistrationSolution& solution,
                                              NodeId query_agent_id,
                                              NodeId match_root) {
  if (!solution.valid || solution.inliers.empty()) {
    return {};
  }

  auto from_pose_info = getQueryPose(graph, query_agent_id);
  auto to_pose_info = getAgentPose(graph, match_root);

  if (!from_pose_info.valid || !to_pose_info.valid) {
    return {};
  }

  // The layer solver returns dest_T_src between the two place subgraphs in the
  // world frame. Compose with the query and match agent poses to express the
  // closure as a relative transform between the two agent bodies (to_T_from),
  // which is what the backend factor expects.
  gtsam::Pose3 to_T_from = to_pose_info.world_T_body.inverse() * solution.dest_T_src *
                           from_pose_info.world_T_body;

  VLOG(3) << "[DSG_LCD] Solution of " << NodeSymbol(query_agent_id).getLabel() << " -> "
          << NodeSymbol(to_pose_info.id).getLabel() << ":";
  VLOG(3) << "  - world_T_from: " << getPoseRepr(from_pose_info.world_T_body);
  VLOG(3) << "  - world_T_to:   " << getPoseRepr(to_pose_info.world_T_body);
  VLOG(3) << "  - dest_T_src:   " << getPoseRepr(solution.dest_T_src);
  VLOG(3) << "  - to_T_from:    " << getPoseRepr(to_T_from);

  return {true,
          from_pose_info.id,
          to_pose_info.id,
          to_T_from.translation(),
          to_T_from.rotation().toQuaternion(),
          -1};
}

void logRegistrationProblem(const std::string& path_prefix,
                            const DynamicSceneGraph& dsg,
                            const LayerRegistrationSolution& solution,
                            const DsgRegistrationInput& match,
                            NodeId query_agent_id) {
  static size_t registration_index = 0;
  std::stringstream ss;
  ss << path_prefix << registration_index << ".json";
  ++registration_index;

  std::ofstream outfile(ss.str());
  outfile << "query_id: " << query_agent_id << std::endl;
  outfile << "query_set: " << displayNodeSymbolContainer(match.query_nodes)
          << std::endl;
  outfile << "match_set: " << displayNodeSymbolContainer(match.match_nodes)
          << std::endl;

  auto match_pose_info = getAgentPose(dsg, match.match_root);
  outfile << "match_id: " << match_pose_info.id << std::endl;
  outfile << "world_q_match: " << match_pose_info.world_T_body.rotation().toQuaternion()
          << std::endl;
  outfile << "world_t_match: " << match_pose_info.world_T_body.translation()
          << std::endl;
  outfile << "match_valid: " << match_pose_info.valid << std::endl;

  outfile << "solution_valid: " << solution.valid << std::endl;
  outfile << "dest_q_src: " << solution.dest_T_src.rotation().toQuaternion()
          << std::endl;
  outfile << "dest_t_src: " << solution.dest_T_src.translation() << std::endl;

  // TODO: output position data
}

DsgTeaserSolver::DsgTeaserSolver(LayerId layer_id,
                                 const LayerRegistrationConfig& config,
                                 const TeaserParams& params)
    : layer_id(layer_id), config(config), solver(params) {
  const std::string layer_str = DsgLayers::LayerIdToString(layer_id);
  timer_prefix = "lcd/" + layer_str + "_registration";
  log_prefix = config.registration_output_path + "/" + layer_str + "_registration_";
}

RegistrationSolution DsgTeaserSolver::solve(const DynamicSceneGraph& dsg,
                                            const DsgRegistrationInput& match,
                                            NodeId query_agent_id) const {
  // TODO: helper function in dsg
  const uint64_t timestamp = dsg.getNode(query_agent_id).timestamp.value().count();
  ScopedTimer timer(timer_prefix, timestamp, true, 2, false);

  // Assemble the correspondence problem. Either re-extract fresh subgraphs
  // around the two roots (so the registration node sets can differ from the
  // ones used for matching) or reuse the node sets carried in the match.
  LayerRegistrationProblem<std::set<NodeId>> problem;
  if (config.recreate_subgraph) {
    const bool is_places = layer_id == DsgLayers::PLACES;
    problem.src_nodes =
        getSubgraphNodes(config.subgraph_extraction, dsg, match.query_root, is_places);
    problem.dest_nodes =
        getSubgraphNodes(config.subgraph_extraction, dsg, match.match_root, is_places);
  } else {
    problem.src_nodes = match.query_nodes;
    problem.dest_nodes = match.match_nodes;
  }

  // TEASER needs enough points to solve; bail out if either side is too small.
  if (problem.src_nodes.size() <= 3 || problem.dest_nodes.size() <= 3) {
    if (problem.src_nodes.empty()) {
      LOG(ERROR) << "Invalid query: " << NodeSymbol(match.query_root).getLabel();
    } else {
      LOG(ERROR) << "Invalid match: " << NodeSymbol(match.match_root).getLabel();
    }

    return {};
  }

  // If the two subgraphs share too many nodes they describe the same region;
  // a registration between them would be near-identity and uninformative.
  size_t num_same = 0;
  for (const auto& qnode : problem.src_nodes) {
    if (problem.dest_nodes.count(qnode)) {
      num_same++;
    }
  }

  if (num_same >= config.max_same_nodes) {
    VLOG(2) << "Rejecting registration: " << num_same << " / " << config.max_same_nodes
            << " shared nodes";
    return {};
  }

  // Build correspondences and run the robust solver. Pairwise matching forms
  // all-pairs correspondences; semantic matching only pairs nodes of the same
  // class to cut down the correspondence count.
  const auto& layer = dsg.getLayer(layer_id);
  LayerRegistrationSolution solution;
  if (config.use_pairwise_registration) {
    solution = registerDsgLayerPairwise(config, solver, problem, layer);
  } else {
    solution = registerDsgLayerSemantic(config, solver, problem, layer);
  }

  if (config.log_registration_problem) {
    logRegistrationProblem(log_prefix, dsg, solution, match, query_agent_id);
  }

  return getFullSolutionFromLayer(dsg, solution, query_agent_id, match.match_root);
}

}  // namespace hydra::lcd

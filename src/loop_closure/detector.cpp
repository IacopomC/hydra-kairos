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
#include "hydra/loop_closure/detector.h"

#include "hydra_build_config.h"
#if defined(HYDRA_USE_GNN) && HYDRA_USE_GNN
#include "hydra/loop_closure/gnn_descriptors.h"
#endif

#include <glog/logging.h>

#include <fstream>
#include <sstream>

#include "hydra/utils/timing_utilities.h"

namespace hydra::lcd {

template <typename Scalar>
std::string showVector(const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& vector,
                       int precision = 3,
                       int max_size = 8,
                       int num_to_show = 3) {
  std::stringstream ss;
  if (vector.rows() <= max_size) {
    const Eigen::IOFormat format(
        precision, Eigen::DontAlignCols, ", ", ", ", "", "", "[", "]");
    ss << vector.format(format);
    return ss.str();
  }

  const Eigen::IOFormat format(
      precision, Eigen::DontAlignCols, ", ", ", ", "", "", "", "");

  ss << "[" << vector.head(num_to_show).format(format) << ", ..., "
     << vector.tail(num_to_show).format(format) << "]";
  return ss.str();
}

#if defined(HYDRA_USE_GNN) && HYDRA_USE_GNN
void configureDescriptorFactories(lcd::LcdDetector& detector,
                                  const LcdDetectorConfig& config) {
  ObjectGnnDescriptor::LabelEmbeddings embeddings;
  if (config.gnn_lcd.use_onehot_encoding) {
    for (size_t i = 0; i < config.gnn_lcd.onehot_encoding_dim; ++i) {
      Eigen::VectorXf vec = Eigen::VectorXf::Zero(config.gnn_lcd.onehot_encoding_dim);
      vec(i) = 1.0f;
      embeddings[static_cast<uint8_t>(i)] = vec;
    }
  } else {
    embeddings = loadLabelEmbeddings(config.gnn_lcd.label_embeddings_file);
  }

  LcdDetector::FactoryMap factories;
  factories.emplace(
      DsgLayers::OBJECTS,
      std::make_unique<ObjectGnnDescriptor>(config.gnn_lcd.object_model_path,
                                            config.object_extraction,
                                            config.gnn_lcd.object_connection_radius_m,
                                            embeddings,
                                            config.gnn_lcd.objects_pos_in_feature));
  factories.emplace(
      DsgLayers::PLACES,
      std::make_unique<PlaceGnnDescriptor>(config.gnn_lcd.places_model_path,
                                           config.places_extraction,
                                           config.gnn_lcd.places_pos_in_feature));
  detector.setDescriptorFactories(std::move(factories));
}
#else
void configureDescriptorFactories(lcd::LcdDetector&, const LcdDetectorConfig&) {
  LOG(ERROR) << "Unable to initialize GNN descriptors: not built with -DHYDRA_GNN=ON";
}
#endif

using hydra::timing::ScopedTimer;
using SearchConfigMap = std::map<LayerId, DescriptorMatchConfig>;
using RegConfigMap = std::map<LayerId, LayerRegistrationConfig>;

LcdDetector::LcdDetector(const LcdDetectorConfig& config) : config_(config) {
  makeDefaultDescriptorFactories();
  if (config_.use_gnn_descriptors) {
    configureDescriptorFactories(*this, config_);
  }

  const SearchConfigMap search_configs{{DsgLayers::OBJECTS, config_.objects.matching},
                                       {DsgLayers::PLACES, config_.places.matching}};
  // Places get a TEASER solver in addition to objects so that offline
  // pipelines (no external frame-registration service available) still
  // produce 6-DoF loop closures via geometric registration on place
  // correspondences. With objects-only, the level loop in registerAndVerify
  // short-circuits at level 1; with both, level 2 runs only if objects fails.
  const RegConfigMap reg_configs{{DsgLayers::OBJECTS, config_.objects.registration},
                                 {DsgLayers::PLACES, config_.places.registration}};
  resetLayerAssignments(search_configs, reg_configs);
}

void LcdDetector::setDescriptorFactories(FactoryMap&& factories) {
  layer_factories_ = std::move(factories);
  const SearchConfigMap search_configs{{DsgLayers::OBJECTS, config_.objects.matching},
                                       {DsgLayers::PLACES, config_.places.matching}};
  // Places get a TEASER solver in addition to objects so that offline
  // pipelines (no external frame-registration service available) still
  // produce 6-DoF loop closures via geometric registration on place
  // correspondences. With objects-only, the level loop in registerAndVerify
  // short-circuits at level 1; with both, level 2 runs only if objects fails.
  const RegConfigMap reg_configs{{DsgLayers::OBJECTS, config_.objects.registration},
                                 {DsgLayers::PLACES, config_.places.registration}};
  resetLayerAssignments(search_configs, reg_configs);
}

void LcdDetector::setRegistrationSolver(size_t level,
                                        DsgRegistrationSolver::Ptr&& solver) {
  if (level == 0 && !config_.enable_agent_registration) {
    return;
  }

  registration_solvers_[level] = std::move(solver);
}

size_t LcdDetector::numDescriptors() const {
  size_t num_descriptors = 0;
  for (const auto& id_cache_pair : cache_map_) {
    num_descriptors += id_cache_pair.second.size();
  }
  return num_descriptors + numAgentDescriptors();
}

size_t LcdDetector::numGraphDescriptors(LayerId layer) const {
  if (!cache_map_.count(layer)) {
    return 0;
  }

  return cache_map_.at(layer).size();
}

size_t LcdDetector::numAgentDescriptors() const {
  size_t count = 0;
  for (const auto& id_cache_pair : leaf_cache_) {
    count += id_cache_pair.second.size();
  }
  return count;
}

const std::map<size_t, LayerSearchResults>& LcdDetector::getLatestMatches() const {
  return matches_;
}

const std::map<LayerId, size_t>& LcdDetector::getLayerRemapping() const {
  return layer_to_internal_index_;
}

const DescriptorCache& LcdDetector::getDescriptorCache(LayerId layer) {
  return cache_map_.at(layer);
}

void LcdDetector::dumpDescriptors(const std::string& log_path) const {
  const Eigen::IOFormat format(
      Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "[", "]");
  std::ofstream fout(log_path + "/descriptors.yaml");
  fout << "---" << std::endl;
  fout << "layers:";
  if (cache_map_.empty()) {
    fout << " []" << std::endl;
    return;
  }

  fout << std::endl;
  for (const auto& id_cache_pair : cache_map_) {
    fout << "  - layer_id: " << id_cache_pair.first << std::endl;
    if (id_cache_pair.second.empty()) {
      fout << "  layer_descriptors: []" << std::endl;
      continue;
    }

    fout << "    layer_descriptors:" << std::endl;
    for (const auto& root_descriptor_pair : id_cache_pair.second) {
      if (!root_descriptor_pair.second) {
        continue;
      }

      if (root_descriptor_pair.second->is_null) {
        continue;
      }

      fout << "      - {root: " << root_descriptor_pair.first
           << ", values: " << root_descriptor_pair.second->values.format(format) << "}"
           << std::endl;
    }
  }
}

void LcdDetector::makeDefaultDescriptorFactories() {
  layer_factories_.emplace(
      DsgLayers::OBJECTS,
      std::make_unique<ObjectDescriptorFactory>(config_.object_extraction,
                                                config_.num_semantic_classes));
  layer_factories_.emplace(
      DsgLayers::PLACES,
      std::make_unique<PlaceDescriptorFactory>(config_.places_extraction,
                                               config_.place_histogram_config));
  agent_factory_ = std::make_unique<AgentDescriptorFactory>();
}

void LcdDetector::resetLayerAssignments(const SearchConfigMap& search_configs,
                                        const RegConfigMap& registration_configs) {
  // TODO: this is messy
  registration_solvers_.clear();

  // Assign each DSG layer a dense internal index starting at 1 (index 0 is
  // reserved for the agent layer). The cascade in detect()/registerAndVerify()
  // walks these indices, so the mapping in both directions is recorded here.
  size_t internal_idx = 1;  // agent is 0
  for (auto&& [layer, layer_config] : search_configs) {
    if (!layer_factories_.count(layer)) {
      continue;
    }

    layer_to_internal_index_[layer] = internal_idx;
    internal_index_to_layer_[internal_idx] = layer;
    match_config_map_[internal_idx] = layer_config;

    auto iter = registration_configs.find(layer);
    if (iter != registration_configs.end()) {
      registration_solvers_.emplace(internal_idx,
                                    std::make_unique<DsgTeaserSolver>(
                                        layer, iter->second, config_.teaser_config));
    }

    internal_idx++;
  }

  // TODO: make root layer requirements explicit
  if (internal_idx == 1) {
    LOG(ERROR) << "No DSG descriptor configs found";
  } else {
    max_internal_index_ = internal_idx;
    root_layer_ = internal_index_to_layer_.at(internal_idx - 1);
  }

  match_config_map_[0] = config_.agent_search_config;

  cache_map_.clear();
  for (const auto& id_func_pair : layer_factories_) {
    cache_map_[id_func_pair.first] = DescriptorCache();
  }
}

bool LcdDetector::addNewDescriptors(const DynamicSceneGraph& graph,
                                    const SceneGraphNode& agent_node) {
  auto parent = agent_node.getParent();
  if (!parent) {
    // Agent has no parent place — caller iterates an archived place's
    // children, so this means the child is a dynamic node that was once
    // parented to this place but is currently re-parented elsewhere or
    // detached. Worth knowing because it means the archived place
    // contributed zero descriptors.
    VLOG(3) << "[Hydra LCD] agent " << NodeSymbol(agent_node.id).getLabel()
            << " has no parent — skipping (no descriptor added)";
    return false;
  }

  VLOG(3) << "[Hydra LCD] addNewDescriptors agent="
          << NodeSymbol(agent_node.id).getLabel()
          << " parent=" << NodeSymbol(*parent).getLabel();

  if (!root_leaf_map_.count(*parent)) {
    root_leaf_map_[*parent] = std::set<NodeId>();
  }

  root_leaf_map_[*parent].insert(agent_node.id);

  if (!leaf_cache_.count(*parent)) {
    leaf_cache_[*parent] = DescriptorCache();
  }

  leaf_cache_[*parent][agent_node.id] = agent_factory_->construct(graph, agent_node);

  for (const auto& prefix_func_pair : layer_factories_) {
    if (cache_map_[prefix_func_pair.first].count(*parent)) {
      continue;
    }

    // guaranteed to exist by constructor
    Descriptor::Ptr layer_descriptor =
        prefix_func_pair.second->construct(graph, agent_node);
    cache_map_[prefix_func_pair.first][*parent] = std::move(layer_descriptor);
  }

  return true;
}

void LcdDetector::updateDescriptorCache(
    const DynamicSceneGraph& dsg,
    const std::unordered_set<NodeId>& archived_places,
    uint64_t timestamp) {
  ScopedTimer timer("lcd/update_descriptors", timestamp, true, 2, false);

  // Per-place children diagnostic: tells us why cache_map_ stays empty in
  // some scenes — if archived places have zero dynamic (agent) children,
  // addNewDescriptors is never called and cache_map_ never grows.
  size_t total_children = 0;
  size_t total_dynamic = 0;
  size_t places_missing = 0;
  size_t places_with_no_dynamic = 0;
  std::set<NodeId> new_agent_nodes;
  for (const auto& place_id : archived_places) {
    if (!dsg.hasNode(place_id)) {
      ++places_missing;
      continue;  // ideally this doesn't happen in practice, but worth building in the
                 // sanity check for now
    }

    const auto& node = dsg.getNode(place_id);

    size_t place_children = 0;
    size_t place_dynamic = 0;
    for (const auto& child : node.children()) {
      ++place_children;
      if (dsg.isDynamic(child)) {
        ++place_dynamic;
        new_agent_nodes.insert(child);
      }
    }
    total_children += place_children;
    total_dynamic += place_dynamic;
    if (place_dynamic == 0) {
      ++places_with_no_dynamic;
    }
  }

  // Snapshot cache sizes before so we can report the delta this call.
  std::map<LayerId, size_t> sizes_before;
  for (const auto& [layer, cache] : cache_map_) {
    sizes_before[layer] = cache.size();
  }

  for (const auto& agent_node : new_agent_nodes) {
    const auto& node = dsg.getNode(agent_node);
    addNewDescriptors(dsg, node);
  }

  // Report the outcome so the next run answers "did caching actually work?"
  std::stringstream layer_sizes;
  for (const auto& [layer, cache] : cache_map_) {
    const size_t before = sizes_before.count(layer) ? sizes_before[layer] : 0;
    layer_sizes << " layer=" << layer << " " << before << "->" << cache.size();
  }
  LOG(INFO) << "[Hydra LCD] updateDescriptorCache:"
            << " archived=" << archived_places.size()
            << " missing=" << places_missing
            << " no_dynamic_children=" << places_with_no_dynamic
            << " children_total=" << total_children
            << " dynamic_total=" << total_dynamic
            << " new_agents=" << new_agent_nodes.size() << " cache:"
            << layer_sizes.str();
}

std::vector<RegistrationSolution> LcdDetector::registerAndVerify(
    const DynamicSceneGraph& dsg,
    const std::map<size_t, LayerSearchResults>& matches,
    NodeId agent_id,
    uint64_t timestamp) const {
  ScopedTimer timer("lcd/register", timestamp, true, 2, false);

  RegistrationSolution registration_result;

  // Registration cascade: walk layers from the agent layer (idx 0) upward and
  // try to register against that layer's matches. The first layer that yields
  // a valid solution wins and the cascade stops, so cheaper/lower layers get
  // priority over geometric place registration above.
  size_t idx;
  for (idx = 0; idx < max_internal_index_; ++idx) {
    if (!registration_solvers_.count(idx)) {
      continue;
    }

    if (!matches.count(idx)) {
      continue;
    }

    CHECK(match_config_map_.count(idx));
    const LayerSearchResults& match = matches.at(idx);
    if (match.valid_matches.empty()) {
      continue;
    }

    // Try each registration candidate in score order; the candidates are
    // sorted descending, so a sub-threshold score means none remain.
    for (size_t i = 0; i < match.match_root.size(); i++) {
      if (match.score[i] < match_config_map_.at(idx).min_registration_score) {
        break;
      }

      const auto root = match.match_root.at(i);
      CHECK(dsg.hasNode(root)) << "Invalid match root: " << NodeSymbol(root).getLabel();
      CHECK(dsg.hasNode(match.query_root))
          << "Invalid query root: " << NodeSymbol(match.query_root).getLabel();

      DsgRegistrationInput registration_input = {match.query_nodes,
                                                 match.match_nodes[i],
                                                 match.query_root,
                                                 match.match_root[i]};
      registration_result =
          registration_solvers_.at(idx)->solve(dsg, registration_input, agent_id);
      registration_result.level = static_cast<int64_t>(idx);
      if (registration_result.valid) {
        break;
      }
    }

    if (registration_result.valid) {
      break;
    }
  }

  // TODO: fix validation
  // size_t vidx = idx;
  /*++vidx;  // start validation at next layer up*/
  /*for (; vidx < max_internal_index_; ++vidx) {*/
  /*if (!validation_funcs_.count(vidx)) {*/
  /*continue;*/
  /*}*/

  /*CHECK(matches.count(idx));*/
  /*const LayerSearchResults& vmatch = matches.at(vidx);*/

  /*if (!validation_funcs_.at(vidx)(dsg, vmatch)) {*/
  /*registration_result.valid = false;*/
  /*break;*/
  /*}*/
  /*}*/

  std::vector<RegistrationSolution> results;
  if (registration_result.valid) {
    results.push_back(registration_result);
  }

  return results;
}

std::vector<RegistrationSolution> LcdDetector::detect(const DynamicSceneGraph& dsg,
                                                      NodeId agent_id,
                                                      uint64_t timestamp) {
  ScopedTimer timer("lcd/detect", timestamp, true, 2, false);

  // Drop cache entries whose nodes have been removed from the DSG
  // (typically via backend node-merging). Otherwise stale IDs are
  // returned as matches and trip the dsg.hasNode CHECK in
  // registerAndVerify.
  size_t pruned_layer = 0, pruned_leaf = 0, pruned_root = 0;
  for (auto& [layer, cache] : cache_map_) {
    for (auto it = cache.begin(); it != cache.end();) {
      if (!dsg.hasNode(it->first)) {
        it = cache.erase(it);
        ++pruned_layer;
      } else {
        ++it;
      }
    }
  }
  for (auto it = leaf_cache_.begin(); it != leaf_cache_.end();) {
    if (!dsg.hasNode(it->first)) {
      it = leaf_cache_.erase(it);
      ++pruned_leaf;
    } else {
      ++it;
    }
  }
  for (auto it = root_leaf_map_.begin(); it != root_leaf_map_.end();) {
    if (!dsg.hasNode(it->first)) {
      it = root_leaf_map_.erase(it);
      ++pruned_root;
    } else {
      ++it;
    }
  }
  if (pruned_layer || pruned_leaf || pruned_root) {
    LOG(INFO) << "[Hydra LCD] pruned stale cache entries: layer=" << pruned_layer
              << " leaf=" << pruned_leaf << " roots=" << pruned_root;
  }

  std::set<NodeId> prev_valid_roots;
  for (const auto& id_desc_pair : cache_map_[root_layer_]) {
    prev_valid_roots.insert(id_desc_pair.first);
  }

  const auto& latest_node = dsg.getNode(agent_id);
  VLOG(2) << "************************************************************";
  VLOG(2) << "LCD Matching: " << NodeSymbol(latest_node.id).getLabel();

  // Search cascade: starting from the root (highest) layer and working down,
  // build the query descriptor for each layer and match it against that
  // layer's cache, restricted to the roots that survived the layer above.
  // This progressively narrows the candidate roots before the agent search.
  matches_.clear();
  for (size_t idx = max_internal_index_ - 1; idx > 0; --idx) {
    const auto& config = match_config_map_.at(idx);
    const LayerId layer = internal_index_to_layer_.at(idx);
    auto descriptor = layer_factories_[layer]->construct(dsg, latest_node);
    if (descriptor) {
      VLOG(2) << "level " << idx << ": " << showVector(descriptor->values, 3, 20, 9);
      matches_[idx] = searchDescriptors(*descriptor,
                                        config,
                                        prev_valid_roots,
                                        cache_map_[layer],
                                        root_leaf_map_,
                                        agent_id);
      // Only narrow prev_valid_roots when this layer actually contributed
      // matches. An intermediate layer with an empty descriptor (e.g. an
      // OBJECTS layer in a scene with no objects) would otherwise clobber
      // prev_valid_roots with an empty set, starving the agent-layer search
      // of candidates and short-circuiting BowAgentSolver. With this guard an
      // empty layer just doesn't refine; the agent search still inherits the
      // last layer that did match (typically PLACES).
      if (!matches_[idx].valid_matches.empty()) {
        prev_valid_roots = matches_[idx].valid_matches;
      }
    } else {
      VLOG(2) << "level " << idx << " -> ?";
      // Descriptor construction itself failed (vs. matching producing
      // zero results); keep the cascade conservative and stop.
      prev_valid_roots = std::set<NodeId>();
      break;
    }
  }

  // Agent layer (index 0): match the query's BoW descriptor against the agent
  // leaves under the roots that survived the cascade above.
  auto descriptor = agent_factory_->construct(dsg, latest_node);
  if (descriptor) {
    matches_[0] = searchLeafDescriptors(*descriptor,
                                        config_.agent_search_config,
                                        prev_valid_roots,
                                        leaf_cache_,
                                        agent_id);
  }

  if (matches_.empty()) {
    VLOG(1) << "No LCD matches for node " << NodeSymbol(agent_id).getLabel()
            << " against " << numDescriptors() << " descriptors";
    return {};
  }

  // Promoted to INFO so we can see the per-layer match counts in the run
  // log without bumping verbosity. This is what tells us whether the
  // agent layer (idx=0) is even getting candidates fed to BowAgentSolver.
  // Once tuned, downgrade back to VLOG(2).
  LOG(INFO) << "[Hydra LCD] results for node " << NodeSymbol(agent_id).getLabel()
            << " against " << numDescriptors() / 2 << " roots";
  for (const auto& id_match_pair : matches_) {
    const auto& scores = id_match_pair.second.score;
    const auto best_score = std::max_element(scores.begin(), scores.end());
    LOG(INFO) << "[Hydra LCD]  - index " << id_match_pair.first << ": "
              << id_match_pair.second.valid_matches.size() << " valid matches, "
              << id_match_pair.second.match_root.size()
              << " registration matches (best score "
              << (best_score != scores.end() ? std::to_string(*best_score) : "n/a")
              << ")";

    for (size_t i = 0; i < id_match_pair.second.match_root.size(); ++i) {
      VLOG(2) << "   - #" << i
              << ": query=" << NodeSymbol(id_match_pair.second.query_root).getLabel()
              << ", match=" << NodeSymbol(id_match_pair.second.match_root[i]).getLabel()
              << " -> " << id_match_pair.second.score[i];
    }
  }

  VLOG(2) << "-----------------------------------------------------------";
  const auto results = registerAndVerify(dsg, matches_, agent_id, timestamp);
  VLOG(2) << "************************************************************";

  return results;
}

}  // namespace hydra::lcd

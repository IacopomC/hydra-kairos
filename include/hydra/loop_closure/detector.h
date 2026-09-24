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
 * @file detector.h
 * @brief Loop-closure detector. Caches a descriptor per node across layers,
 *        searches those caches for matches to a query agent, and runs the
 *        registration cascade from the agent layer down to objects and places
 *        to produce loop-closure constraints.
 */
#pragma once
#include "hydra/loop_closure/descriptor_matching.h"
#include "hydra/loop_closure/registration.h"
#include "hydra/loop_closure/scene_graph_descriptors.h"

namespace hydra::lcd {

/// @brief Settings for the graph-network descriptor variants.
struct GnnLcdConfig {
  bool use_onehot_encoding = true;     ///< Encode labels as one-hot vectors instead of loading embeddings.
  size_t onehot_encoding_dim = 39;     ///< Dimension of the one-hot label encoding.
  std::string label_embeddings_file;   ///< File of label embeddings, used when not one-hot encoding.
  double object_connection_radius_m;   ///< Maximum spacing for an edge in the object input graph (m).
  std::string object_model_path;       ///< Path to the object descriptor model.
  std::string places_model_path;       ///< Path to the place descriptor model.
  bool places_pos_in_feature = false;  ///< Append node position to place node features.
  bool objects_pos_in_feature = false; ///< Append node position to object node features.
};

/// @brief Matching and registration settings for one scene-graph layer.
struct LayerLcdConfig {
  DescriptorMatchConfig matching;        ///< Descriptor search thresholds for the layer.
  LayerRegistrationConfig registration;  ///< Registration thresholds for the layer.
};

/// @brief Full configuration of the loop-closure detector.
struct LcdDetectorConfig {
  TeaserParams teaser_config;                  ///< Robust registration solver parameters.
  bool enable_agent_registration = true;       ///< Run an agent-layer solver before lower layers when true.
  DescriptorMatchConfig agent_search_config;    ///< Descriptor search thresholds for the agent layer.

  LayerLcdConfig objects;  ///< Object-layer matching and registration settings.
  LayerLcdConfig places;   ///< Place-layer matching and registration settings.

  SubgraphConfig object_extraction{5.0};  ///< Subgraph extraction for object descriptors.
  SubgraphConfig places_extraction{5.0};  ///< Subgraph extraction for place descriptors.

  size_t num_semantic_classes = 20;  ///< Number of semantic classes for object descriptors.
  HistogramConfig<double> place_histogram_config{0.5, 2.5, 30};  ///< Binning for place descriptors.
  bool use_gnn_descriptors = false;  ///< Use graph-network descriptors in place of the default ones.
  GnnLcdConfig gnn_lcd;              ///< Graph-network descriptor settings.
};

/// @brief Detects loop closures by caching per-node descriptors, searching them
/// for matches to a query agent, and registering the matched subgraphs.
///
/// Layers are assigned ascending internal indices, with the agent layer at
/// index zero. Detection searches each layer from the top down for candidate
/// matches, then registration runs from low indices upward and returns the
/// first valid solution.
class LcdDetector {
 public:
  using FactoryMap = std::map<LayerId, DescriptorFactory::Ptr>;  ///< Descriptor factories keyed by layer.
  using SearchResultMap = std::map<size_t, LayerSearchResults>;  ///< Search results keyed by internal layer index.

  /// @brief Construct the detector and set up its default descriptor factories and solvers.
  /// @param config Detector configuration.
  explicit LcdDetector(const LcdDetectorConfig& config);

  /// @brief Replace the per-layer descriptor factories.
  /// @param factories Factories to take ownership of, keyed by layer.
  void setDescriptorFactories(FactoryMap&& factories);

  /// @brief Install a registration solver for one internal layer index.
  /// @param level Internal layer index.
  /// @param solver Solver to take ownership of.
  void setRegistrationSolver(size_t level, DsgRegistrationSolver::Ptr&& solver);

  /// @brief Refresh cached descriptors for newly archived places and agent nodes.
  /// @param dsg Scene graph to read descriptors from.
  /// @param archived_places Place nodes finalized this update.
  /// @param timestamp Update timestamp in nanoseconds.
  void updateDescriptorCache(const DynamicSceneGraph& dsg,
                             const std::unordered_set<NodeId>& archived_places,
                             uint64_t timestamp = 0);

  /// @brief Search for and register loop closures against a query agent.
  /// @param dsg Scene graph holding the cached and query nodes.
  /// @param latest_agent_id Query agent to look for revisits against.
  /// @param timestamp Detection timestamp in nanoseconds.
  /// @return Valid loop-closure solutions, empty when none are found.
  std::vector<RegistrationSolution> detect(const DynamicSceneGraph& dsg,
                                           NodeId latest_agent_id,
                                           uint64_t timestamp = 0);

  /// @brief Total number of cached descriptors across all layers and agents.
  /// @return The descriptor count.
  size_t numDescriptors() const;

  /// @brief Number of cached descriptors for one layer.
  /// @param layer Layer to count.
  /// @return The descriptor count, zero when the layer has no cache.
  size_t numGraphDescriptors(LayerId layer) const;

  /// @brief Number of cached agent-layer descriptors.
  /// @return The agent descriptor count.
  size_t numAgentDescriptors() const;

  /// @brief Search results from the most recent detect() call.
  /// @return Per-layer search results.
  const SearchResultMap& getLatestMatches() const;

  /// @brief Mapping from layer identifier to internal layer index.
  /// @return The layer-to-index map.
  const std::map<LayerId, size_t>& getLayerRemapping() const;

  /// @brief Access the descriptor cache for one layer.
  /// @param layer Layer to access.
  /// @return The descriptor cache for the layer.
  const DescriptorCache& getDescriptorCache(LayerId layer);

  /// @brief Write the cached descriptors to a file for inspection.
  /// @param log_path Directory the descriptor dump is written to.
  void dumpDescriptors(const std::string& log_path) const;

 protected:
  /// @brief Populate the built-in agent, object, and place descriptor factories.
  void makeDefaultDescriptorFactories();

  /// @brief Assign layers to internal indices and create their match configs and solvers.
  /// @param match_configs Per-layer descriptor match settings.
  /// @param reg_configs Per-layer registration settings.
  void resetLayerAssignments(
      const std::map<LayerId, DescriptorMatchConfig>& match_configs,
      const std::map<LayerId, LayerRegistrationConfig>& reg_configs);

  /// @brief Add descriptors for an agent node and its associated subgraphs to the caches.
  /// @param graph Scene graph to read from.
  /// @param agent_node Agent node to describe.
  /// @return True when a descriptor was added.
  bool addNewDescriptors(const DynamicSceneGraph& graph,
                         const SceneGraphNode& agent_node);

  /// @brief Run the registration cascade over candidate matches and keep valid solutions.
  /// @param dsg Scene graph supplying node positions.
  /// @param matches Per-layer candidate matches.
  /// @param agent_node Query agent the closures anchor to.
  /// @param timestamp Detection timestamp in nanoseconds.
  /// @return The first valid solution found, or empty.
  std::vector<RegistrationSolution> registerAndVerify(const DynamicSceneGraph& dsg,
                                                      const SearchResultMap& matches,
                                                      NodeId agent_node,
                                                      uint64_t timestamp = 0) const;

  LcdDetectorConfig config_;            ///< Detector configuration.
  DescriptorFactory::Ptr agent_factory_;///< Factory for agent-layer descriptors.
  FactoryMap layer_factories_;          ///< Descriptor factories keyed by layer.

  LayerId root_layer_;                  ///< Highest layer searched, iterated first during detection.
  size_t max_internal_index_;           ///< One past the highest internal layer index.
  std::map<LayerId, size_t> layer_to_internal_index_;  ///< Layer identifier to internal index.
  std::map<size_t, LayerId> internal_index_to_layer_;  ///< Internal index to layer identifier.

  std::map<size_t, DescriptorMatchConfig> match_config_map_;      ///< Match settings keyed by internal index.
  std::map<size_t, DsgRegistrationSolver::Ptr> registration_solvers_;  ///< Solvers keyed by internal index.

  std::map<LayerId, DescriptorCache> cache_map_;       ///< Root descriptors keyed by layer.
  std::map<NodeId, DescriptorCache> leaf_cache_;       ///< Per-root caches of leaf descriptors.
  std::map<NodeId, std::set<NodeId>> root_leaf_map_;   ///< Leaf nodes grouped under each root.

  std::map<size_t, LayerSearchResults> matches_;       ///< Search results from the latest detection.
};

}  // namespace hydra::lcd

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
#include <spark_dsg/layer_prefix.h>
#include <spark_dsg/scene_graph_types.h>

#include <map>
#include <memory>
#include <set>
#include <vector>

namespace spark_dsg {
class DynamicSceneGraph;
}

/**
 * @file graph_connector.h
 * @brief Maintains parent/child interlayer edges in the scene graph, attaching
 *        each child node to its nearest active parent.
 */

namespace hydra {

/// @brief Connects nodes in one set of child layers to their nearest parent in
/// a single parent layer. Tracks which parents and children are active and
/// rewires edges as nodes appear or become archived.
struct LayerConnector {
  /// @brief Settings for one parent layer and the child layers attached to it.
  struct Config {
    /// @brief Selects a child layer and whether its static/dynamic nodes participate.
    struct ChildLayerConfig {
      spark_dsg::LayerId layer = spark_dsg::DsgLayers::UNKNOWN;  ///< Child layer identifier.
      bool include_static = true;   ///< Connect static nodes of this layer.
      bool include_dynamic = true;  ///< Connect dynamic nodes of this layer.
    };

    spark_dsg::LayerId parent_layer = spark_dsg::DsgLayers::PLACES;  ///< Parent layer; must be static.
    std::vector<ChildLayerConfig> child_layers{
        {spark_dsg::DsgLayers::OBJECTS, true, true}};  ///< Child layers to attach to the parent.
    size_t verbosity = 0;  ///< Logging verbosity for this connector.
  } const config;

  /// @brief Construct from config and precompute the static/dynamic child sets.
  explicit LayerConnector(const Config& config);

  /// @brief Test whether a layer key belongs to a configured child layer.
  bool isChild(const spark_dsg::LayerKey& key) const;

  /// @brief Add new parent nodes to tracking and drop parents that became
  /// archived or were removed, releasing their children.
  /// @param graph Scene graph being inspected.
  /// @param new_nodes Nodes added since the previous call.
  void updateParents(const spark_dsg::DynamicSceneGraph& graph,
                     const std::vector<spark_dsg::NodeId>& new_nodes);

  /// @brief Track new child nodes and connect each active child to its nearest
  /// active parent, replacing any prior parent edge.
  /// @param graph Scene graph whose edges are updated in place.
  /// @param new_nodes Nodes added since the previous call.
  void connectChildren(spark_dsg::DynamicSceneGraph& graph,
                       const std::vector<spark_dsg::NodeId>& new_nodes);

  const spark_dsg::LayerKey parent_layer;                     ///< Resolved parent layer key.
  std::set<spark_dsg::LayerId> static_child_layers;           ///< Child layers whose static nodes connect.
  std::set<spark_dsg::LayerId> dynamic_child_layers;          ///< Child layers whose dynamic nodes connect.
  std::set<spark_dsg::NodeId> active_children;                ///< Currently tracked child nodes.
  std::map<spark_dsg::NodeId, std::set<spark_dsg::NodeId>>
      active_parents;  ///< Active parents and the children attached to each.
};

/// @brief Declare the config fields for a child-layer entry.
void declare_config(LayerConnector::Config::ChildLayerConfig& config);

/// @brief Declare the config fields for a layer connector.
void declare_config(LayerConnector::Config& config);

/// @brief Runs a set of LayerConnectors to maintain all interlayer parent/child
/// edges each time the graph changes.
class GraphConnector {
 public:
  /// @brief Settings listing the per-layer connectors to run.
  struct Config {
    std::vector<LayerConnector::Config> layers{LayerConnector::Config{}};  ///< Connectors to apply.
  } const config;

  /// @brief Construct and validate the configured layer connectors.
  explicit GraphConnector(const Config& config);

  virtual ~GraphConnector();

  /// @brief Update every layer connector against the graph's newly added nodes.
  void connect(spark_dsg::DynamicSceneGraph& graph);

 protected:
  std::vector<LayerConnector> layers_;  ///< Per-layer connectors.
};

/// @brief Declare the config fields for the graph connector.
void declare_config(GraphConnector::Config& config);

}  // namespace hydra

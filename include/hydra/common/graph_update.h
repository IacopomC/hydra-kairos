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
#include <spark_dsg/node_attributes.h>
#include <spark_dsg/node_symbol.h>

#include <list>
#include <map>

/**
 * @file graph_update.h
 * @brief Batched insertion of new nodes into a scene graph. Collects pending
 *        node attributes per layer, tracks the next free symbol id per layer,
 *        and applies the accumulated update to a target graph.
 */

namespace spark_dsg {
class DynamicSceneGraph;
}

namespace hydra {

/// @brief Pending node insertions for a single scene-graph layer.
struct LayerUpdate {
  using Ptr = std::shared_ptr<LayerUpdate>;
  /// @brief Construct an empty update bound to a layer.
  /// @param layer Layer the queued nodes belong to.
  explicit LayerUpdate(spark_dsg::LayerId layer);
  /// @brief Move the node attributes from another update into this one.
  /// @param other Update whose attributes are consumed.
  void append(LayerUpdate&& other);

  const spark_dsg::LayerId layer;  ///< Layer these nodes belong to.
  std::list<spark_dsg::NodeAttributes::Ptr> attributes;  ///< Attributes of the nodes to add.
};

/// @brief A set of per-layer node insertions keyed by layer id.
using GraphUpdate = std::map<spark_dsg::LayerId, LayerUpdate::Ptr>;

/// @brief Allocates successive node symbols for one layer during updates.
struct LayerTracker {
  /// @brief Configuration controlling symbol allocation for a layer.
  struct Config {
    char prefix = 0;  ///< Symbol prefix for ids minted in this layer.
    std::optional<spark_dsg::LayerId> target_layer;  ///< Layer to place new nodes in, if remapped.
  } const config;

  /// @brief Construct a tracker starting from the first id for the prefix.
  /// @param config Allocation settings for the layer.
  explicit LayerTracker(const Config& config);

  spark_dsg::NodeSymbol next_id;  ///< Next symbol to assign to a new node.
};

/// @brief Register the layer tracker config fields with the config system.
void declare_config(LayerTracker::Config& config);

/// @brief Applies batched node insertions to a scene graph.
/// Holds one tracker per configured layer so newly added nodes receive
/// consistent, monotonically increasing symbol ids.
struct GraphUpdater {
  /// @brief Configuration mapping layer names to their tracker settings.
  struct Config {
    std::map<std::string, LayerTracker::Config> layer_updates;  ///< Per-layer tracker settings keyed by layer name.
  } const config;

  /// @brief Construct an updater and its per-layer trackers from config.
  /// @param config Per-layer allocation settings.
  explicit GraphUpdater(const Config& config);
  /// @brief Insert the queued nodes into the graph, assigning ids per layer.
  /// @param update Pending insertions grouped by layer.
  /// @param graph Graph to add the new nodes to.
  void update(const GraphUpdate& update, spark_dsg::DynamicSceneGraph& graph);

 private:
  std::map<spark_dsg::LayerId, LayerTracker> trackers_;  ///< Id allocators keyed by layer.
};

/// @brief Register the graph updater config fields with the config system.
void declare_config(GraphUpdater::Config& config);

}  // namespace hydra

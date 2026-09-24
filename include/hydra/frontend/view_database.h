#pragma once
#include "hydra/common/dsg_types.h"
#include "hydra/frontend/view_selector.h"

/**
 * @file view_database.h
 * @brief Maintains the set of recent sensor views and assigns each active place
 *        node a semantic feature drawn from the views that can see it.
 */

namespace hydra {

struct InputData;

/// @brief Buffers feature views and uses a view selector to populate the
/// semantic feature of active place nodes. Views that no longer see any active
/// place are dropped.
class ViewDatabase {
 public:
  using Ptr = std::shared_ptr<ViewDatabase>;

  /// @brief Settings controlling view-to-node feature assignment.
  struct Config {
    std::string view_selection_method = "boundary";  ///< Registered ViewSelector to instantiate.
  };

  /// @brief Construct the database and create the configured view selector.
  explicit ViewDatabase(const Config& config);

  ~ViewDatabase();

  /// @brief Drain newly received views, prune views with no visible active
  /// place, then assign a feature to each active place node.
  /// @param graph Scene graph providing node positions; features written in place.
  /// @param active_places Nodes whose semantic feature should be updated.
  void updateAssignments(const DynamicSceneGraph& graph,
                         const std::unordered_set<NodeId>& active_places) const;

 protected:
  mutable ViewSelector::FeatureList views_;        ///< Buffered views retained across calls.
  std::unique_ptr<ViewSelector> view_selector_;    ///< Strategy used to pick or fuse features.
};

/// @brief Declare the config fields for config_utilities.
void declare_config(ViewDatabase::Config& config);

}  // namespace hydra

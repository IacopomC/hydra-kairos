#include "hydra/frontend/view_database.h"

#include <config_utilities/config.h>
#include <config_utilities/factory.h>
#include <glog/logging.h>

#include "hydra/common/pipeline_queues.h"
#include "hydra/input/input_data.h"

namespace hydra {

void declare_config(ViewDatabase::Config& config) {
  using namespace config;
  name("ViewDatabase::Config");
  field(config.view_selection_method, "view_selection_method");
}

using NodeSet = std::unordered_set<NodeId>;

ViewDatabase::ViewDatabase(const Config& config)
    : view_selector_(config::create<ViewSelector>(config.view_selection_method)) {
  CHECK(view_selector_);
}

ViewDatabase::~ViewDatabase() {}

void ViewDatabase::updateAssignments(const DynamicSceneGraph& graph,
                                     const NodeSet& active_places) const {
  // drain any feature views produced since the last call
  auto& queue = PipelineQueues::instance().input_features_queue;
  size_t new_views = 0;
  while (!queue.empty()) {
    ++new_views;
    views_.push_back(queue.pop());
  }

  VLOG(2) << "Got " << new_views << " new views!";

  // archive views that no longer see any active place: a view is kept only while at
  // least one active place still falls inside its frustum
  auto iter = views_.begin();
  while (iter != views_.end()) {
    const auto& view = *iter;
    if (!view) {
      iter = views_.erase(iter);
      continue;
    }

    bool visible = false;
    for (const auto node_id : active_places) {
      // TODO: this isn't the best archival logic ever, we probably want some
      // sort of occlusion check
      if (view->pointInView(graph.getNode(node_id).attributes().position)) {
        visible = true;
        break;
      }
    }

    if (!visible) {
      iter = views_.erase(iter);
      continue;
    }

    ++iter;
  }

  if (views_.empty()) {
    VLOG(2) << "No views assigned!";
    return;
  }

  // assign each active place a semantic feature drawn from the surviving views, per the
  // configured selection policy (closest view, boundary view, or fusion)
  VLOG(2) << "Assigning features with " << views_.size() << " active view(s)";
  for (const auto node_id : active_places) {
    auto& attrs = graph.getNode(node_id).attributes<SemanticNodeAttributes>();
    view_selector_->selectFeature(views_, attrs);
  }
}

}  // namespace hydra

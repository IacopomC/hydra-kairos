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
#include "hydra/common/dsg_types.h"
#include "hydra/utils/disjoint_set.h"

/**
 * @file graph_filtration.h
 * @brief Computes a graph filtration over a places layer: as a dilation distance
 *        sweeps from low to high, nodes and edges enter the graph and connected
 *        components merge, yielding a curve of component counts versus distance.
 *        Also provides helpers to track per-component lifetimes (barcodes) and to
 *        pick a representative dilation distance from the resulting curve.
 */

namespace hydra {

/// @brief One sample of the filtration: a component count at a given dilation distance.
struct FiltrationInfo {
  double distance;        ///< Dilation distance at this sample.
  size_t num_components;  ///< Number of components present at this distance.
};

/// @brief Distance interval over which a connected component exists in the filtration.
struct ComponentLifetime {
  double start;  ///< Distance at which the component first appears.
  double end;    ///< Distance at which the component merges away.
};

/// @brief Lifetime of each tracked component, keyed by its representative node ID.
using LifetimeMap = std::unordered_map<NodeId, ComponentLifetime>;

/// @brief Disjoint-set union-find that also records component lifetimes (barcodes)
/// as components form and merge during the filtration sweep.
struct BarcodeTracker : public DisjointSet {
  /// @brief Construct with no minimum component size (tracks every component).
  BarcodeTracker();

  /// @brief Construct tracking only components reaching the given size.
  /// @param min_component_size Smallest component size that gets a lifetime entry.
  explicit BarcodeTracker(size_t min_component_size);

  virtual ~BarcodeTracker() = default;

  /// @brief Register a singleton node entering at the given distance.
  /// @param node Node being added.
  /// @param distance Dilation distance at which the node enters.
  void addNode(NodeId node, double distance);

  /// @brief Merge the components of two nodes, updating the surviving component's
  /// lifetime and closing out the absorbed one.
  /// @param components Disjoint-set being mutated.
  /// @param node_distances Per-node entry distances used to choose the surviving root.
  /// @param node First node of the edge.
  /// @param rhs Second node of the edge.
  /// @param distance Dilation distance at which the merge occurs.
  /// @return True if a merge happened, false if the nodes already shared a component.
  bool doUnion(DisjointSet& components,
               const std::unordered_map<NodeId, double>& node_distances,
               NodeId node,
               NodeId rhs,
               double distance);

  size_t min_component_size;  ///< Smallest component size that receives a lifetime entry.
  LifetimeMap barcodes;       ///< Recorded lifetime per surviving component root.
};

/// @brief Ordered sequence of filtration samples, increasing or decreasing in distance.
using Filtration = std::vector<FiltrationInfo>;
/// @brief Counts the components of interest in a disjoint-set state.
using ComponentCallback = std::function<size_t(const DisjointSet&)>;

/// @brief Stream a single filtration sample as "<dist=..., size=...>".
std::ostream& operator<<(std::ostream& out, const FiltrationInfo& info);

/// @brief Stream a whole filtration as a bracketed list of samples.
std::ostream& operator<<(std::ostream& out, const Filtration& info);

/// @brief Compute the filtration of a layer, counting every component.
/// @param layer Places layer to sweep.
/// @param diff_threshold_m Distances closer than this are merged into one sample.
/// @return The component-count curve over dilation distance.
Filtration getGraphFiltration(const SceneGraphLayer& layer,
                              double diff_threshold_m = 1.0e-4);

/// @brief Compute the filtration of a layer, counting only sufficiently large components.
/// @param layer Places layer to sweep.
/// @param min_component_size Smallest component size that contributes to the count.
/// @param diff_threshold_m Distances closer than this are merged into one sample.
/// @return The component-count curve over dilation distance.
Filtration getGraphFiltration(const SceneGraphLayer& layer,
                              size_t min_component_size,
                              double diff_threshold_m = 1.0e-4);

/// @brief Compute the filtration with full control over component tracking and counting.
/// @param layer Places layer to sweep.
/// @param tracker Barcode tracker that accumulates component lifetimes.
/// @param diff_threshold_m Distances closer than this are merged into one sample.
/// @param count_components Callback returning the component count to record per sample.
/// @param include_nodes If true, nodes enter at their own distance; otherwise all nodes
///        seed the graph up front and only edges drive the sweep.
/// @return The component-count curve over dilation distance.
Filtration getGraphFiltration(const SceneGraphLayer& layer,
                              BarcodeTracker& tracker,
                              double diff_threshold_m,
                              const ComponentCallback& count_components,
                              bool include_nodes = true);

/// @brief Restrict a filtration to the index window covering a dilation range.
/// @param old_filtration Full filtration to trim.
/// @param min_dilation_m Lower bound of the dilation window.
/// @param max_dilation_m Upper bound of the dilation window.
/// @param clip_to_max If the component-count peak falls inside the window, end the window
///        just past that peak rather than at the upper bound.
/// @return The (start, end) index pair bounding the selected window.
std::pair<size_t, size_t> getTrimmedFiltration(const Filtration& old_filtration,
                                               double min_dilation_m,
                                               double max_dilation_m,
                                               bool clip_to_max = true);

/// @brief Find the longest run of consecutive samples that share a nonzero component count.
/// @param values Filtration to scan.
/// @param start_index Inclusive start of the search window.
/// @param end_index Exclusive end of the search window.
/// @return The first sample of the longest run, or nullopt if none qualifies.
std::optional<FiltrationInfo> getLongestSequence(const Filtration& values,
                                                 size_t start_index,
                                                 size_t end_index);

/// @brief Within a window, pick the dilation whose component count matches the dominant
/// count and where the total lifetime of qualifying components is greatest.
/// @param values Filtration to scan.
/// @param lifetimes Component lifetimes from the barcode tracker.
/// @param min_component_lifetime Minimum lifetime length for a component to count.
/// @param start_index Inclusive start of the search window.
/// @param end_index Exclusive end of the search window.
/// @return The chosen sample, or nullopt if none qualifies.
std::optional<FiltrationInfo> getLongestLifetimeDilation(const Filtration& values,
                                                         const LifetimeMap& lifetimes,
                                                         double min_component_lifetime,
                                                         size_t start_index,
                                                         size_t end_index);

/// @brief Within a window, pick the plateau (run of constant component count) with the
/// most components among those whose distance span clears a lifetime threshold.
/// @param values Filtration to scan.
/// @param ratio Either a fraction of the longest plateau span or an absolute threshold.
/// @param start_index Inclusive start of the search window.
/// @param end_index Exclusive end of the search window.
/// @param use_threshold If true, treat ratio as an absolute span threshold; otherwise as
///        a fraction of the longest plateau's span.
/// @return The first sample of the chosen plateau.
std::optional<FiltrationInfo> getBestPlateau(const Filtration& values,
                                             double ratio,
                                             size_t start_index,
                                             size_t end_index,
                                             bool use_threshold = false);

}  // namespace hydra

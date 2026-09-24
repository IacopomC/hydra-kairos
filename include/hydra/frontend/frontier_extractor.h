#pragma once
#include <config_utilities/virtual_config.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <spatial_hash/types.h>

#include "hydra/active_window/active_window_output.h"
#include "hydra/active_window/volumetric_window.h"
#include "hydra/common/dsg_types.h"

/**
 * @file frontier_extractor.h
 * @brief Detects exploration frontiers, the boundary voxels between observed
 *        free space inside places and unobserved space, and inserts them into
 *        the places layer of the scene graph as frontier place nodes.
 */

namespace hydra {

class NearestNodeFinder;

/// @brief A detected frontier: a region on the boundary of unobserved space.
struct Frontier {
 public:
  /// @brief Construct an empty frontier.
  Frontier(){};

  /// @brief Construct a frontier with shape information.
  /// @param c Center position in the world frame.
  /// @param s Per-axis extent of the frontier ellipsoid.
  /// @param o Orientation of the frontier ellipsoid.
  /// @param n Number of frontier voxels making up the region.
  /// @param b Block containing the frontier center.
  Frontier(Eigen::Vector3d c,
           Eigen::Vector3d s,
           Eigen::Quaterniond o,
           size_t n,
           spatial_hash::BlockIndex b)
      : center(c),
        scale(s),
        orientation(o),
        num_frontier_voxels(n),
        block_index(b),
        has_shape_information(true){};

  /// @brief Construct a frontier without shape information.
  /// @param c Center position in the world frame.
  /// @param n Number of frontier voxels making up the region.
  /// @param b Block containing the frontier center.
  Frontier(Eigen::Vector3d c, size_t n, spatial_hash::BlockIndex b)
      : center(c),
        num_frontier_voxels(n),
        block_index(b),
        has_shape_information(false){};

 public:
  Eigen::Vector3d center;                  ///< Center position in the world frame.
  Eigen::Vector3d scale;                   ///< Per-axis extent of the frontier ellipsoid.
  Eigen::Quaterniond orientation;          ///< Orientation of the frontier ellipsoid.
  size_t num_frontier_voxels = 0;          ///< Number of frontier voxels in the region.
  spatial_hash::BlockIndex block_index;    ///< Block containing the frontier center.
  bool has_shape_information = false;      ///< True if scale and orientation are set.
};

/// @brief Finds frontier regions from the active-window TSDF and the current
/// places and adds them to the scene graph as frontier place nodes. Frontiers
/// are voxels that are unobserved, lie inside the reach of a place, and border
/// observed free space; they are either kept dense per voxel or clustered into
/// sparse ellipsoidal regions.
class FrontierExtractor {
 public:
  /// @brief Settings controlling frontier detection and clustering.
  struct Config {
    char prefix = 'f';                       ///< Node id prefix for frontier place nodes.
    double cluster_tolerance = .3;           ///< Max neighbor spacing within a frontier cluster (m).
    size_t min_cluster_size = 10;            ///< Smallest accepted cluster size (voxels).
    size_t max_cluster_size = 100000;        ///< Largest accepted cluster size (voxels).
    double max_place_radius = 5;             ///< Max place reach used to bound the per-block search (m).
    bool dense_frontiers = false;            ///< Keep one node per frontier voxel instead of clustering.
    double frontier_splitting_threshold = 0.2;  ///< Eigenvalue threshold for recursively splitting a cluster.
    size_t point_threshold = 10;             ///< Cluster size below which splitting stops.
    size_t culling_point_threshold = 10;     ///< Discard clusters with fewer voxels than this.
    double recent_block_distance = 25;       ///< Distance beyond which archived blocks are forgotten (m).
    double minimum_relative_z = -0.2;        ///< Lower bound on frontier height relative to the body (m).
    double maximum_relative_z = 1;           ///< Upper bound on frontier height relative to the body (m).
    bool compute_frontier_shape = false;     ///< Fit an oriented ellipsoid to each sparse frontier.
  } const config;

  /// @brief Construct the extractor from its configuration.
  explicit FrontierExtractor(const Config& config);

  /// @brief Archive map blocks that have left the active window and prune
  /// archived blocks beyond the recent-block distance.
  /// @param current_position Current body position in the world frame.
  /// @param block_size Edge length of a map block (m).
  void updateRecentBlocks(const Eigen::Vector3d& current_position, double block_size);

  /// @brief Detect frontier regions for the current step.
  /// @param input Active-window output supplying the latest TSDF.
  /// @param graph Scene graph providing place positions.
  /// @param active_places Place nodes currently active.
  void detectFrontiers(const ActiveWindowOutput& input,
                       DynamicSceneGraph& graph,
                       const NodeIdSet& active_places);

  /// @brief Remove stale frontier nodes and add the latest detected frontiers
  /// to the scene graph, linking each to its nearest place.
  void addFrontiers(uint64_t timestamp_ns, DynamicSceneGraph& graph);

  /// @brief Provide the places archived this step so nearby frontiers can be
  /// flagged as archived.
  void setArchivedPlaces(const std::vector<NodeId>& archived_places);

 private:
  NodeSymbol next_node_id_;                                    ///< Next frontier node id to assign.
  std::vector<std::pair<NodeId, BlockIndex>> nodes_to_remove_; ///< Frontier nodes to remove on the next add.

  TsdfLayer::Ptr tsdf_;                              ///< Local copy of the TSDF accumulated across steps.
  std::vector<NodeId> archived_places_;              ///< Places archived this step.
  spatial_hash::IndexSet just_archived_blocks_;      ///< Blocks that left the window this step.
  spatial_hash::IndexSet recently_archived_blocks_;  ///< Recently archived blocks within the recent distance.
  std::unique_ptr<VolumetricWindow> map_window_;     ///< Window bounds used to decide block archival.

  std::unique_ptr<NearestNodeFinder> place_finder_;  ///< Spatial index over active place nodes.
  std::vector<Frontier> frontiers_;                  ///< Active frontiers detected this step.
  std::vector<Frontier> archived_frontiers_;         ///< Archived frontiers detected this step.

  /// @brief Allocate and copy the latest TSDF blocks into the local layer.
  void updateTsdf(const ActiveWindowOutput& msg);

  /// @brief Emit one frontier per boundary voxel without clustering.
  void populateDenseFrontiers(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                              const pcl::PointCloud<pcl::PointXYZ>::Ptr archived_cloud,
                              const TsdfLayer& layer);

  /// @brief Cluster boundary voxels into sparse frontier regions.
  void computeSparseFrontiers(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                              const TsdfLayer& layer,
                              std::vector<Frontier>& frontiers) const;

  inline static const auto registration_ =
      config::RegistrationWithConfig<FrontierExtractor, FrontierExtractor, Config>(
          "voxel_clustering");  ///< Self-registration under the name "voxel_clustering".
};

/// @brief Compute the mean of a set of positions.
/// @param positions Points to average.
/// @return The centroid of the input positions.
Eigen::Vector3d frontiersToCenters(const std::vector<Eigen::Vector3f>& positions);

/// @brief Declare the config fields for the frontier extractor.
void declare_config(FrontierExtractor::Config& conf);

}  // namespace hydra

#pragma once
#include <pcl/common/centroid.h>
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>

#include "hydra/common/dsg_types.h"
#include "spark_dsg/dynamic_scene_graph.h"
#include "spark_dsg/node_attributes.h"

/**
 * @file place_2d_split_logic.h
 * @brief Geometry for 2D surface places: fitting a bounding ellipse and
 *        boundary, recursively splitting oversized places, and testing whether
 *        two places should be connected by an edge.
 */

namespace hydra {

/// @brief A 2D surface place: a set of mesh vertices with a fitted bounding
/// ellipse, boundary, and the cut used to split it further.
struct Place2d {
  using PointT = spark_dsg::Mesh::Pos;                  ///< Vertex position type.
  using CloudT = spark_dsg::Mesh::Positions;            ///< Container of vertex positions.
  using CentroidT = pcl::CentroidPoint<pcl::PointXYZ>;  ///< Accumulator for the centroid.
  using Index = size_t;                                 ///< Mesh vertex index type.

  CentroidT centroid;                       ///< Running centroid of the member vertices.
  std::vector<Index> indices;               ///< Mesh vertex indices in this place.
  size_t min_mesh_index;                    ///< Lowest mesh vertex index in the place.
  size_t max_mesh_index;                    ///< Highest mesh vertex index in the place.
  std::vector<Index> boundary_indices;      ///< Mesh indices forming the place boundary.
  std::vector<Eigen::Vector3d> boundary;    ///< Boundary points in the world frame.
  Eigen::Matrix2d ellipse_matrix_compress;  ///< Maps a point into the unit-ellipse frame.
  Eigen::Matrix2d ellipse_matrix_expand;    ///< Maps the unit ellipse back to world axes.
  Eigen::Vector2d ellipse_centroid;         ///< Center of the bounding ellipse (xy).
  Eigen::Vector2d cut_plane;                ///< Direction along which the place would split.
  bool can_split;                           ///< Whether the place is still divisible.
};

/// @brief Fit a bounding ellipse to the points and store it in node attributes.
/// @param points Vertex positions for the place.
/// @param connection_ellipse_scale_factor Scale applied to the fitted ellipse.
/// @param attrs Node attributes updated with the ellipse parameters.
void addRectInfo(const Place2d::CloudT& points,
                 const double connection_ellipse_scale_factor,
                 Place2dNodeAttributes& attrs);

/// @brief Fit a bounding ellipse and cut direction to the points and store them.
/// @param points Vertex positions for the place.
/// @param connection_ellipse_scale_factor Scale applied to the fitted ellipse.
/// @param place Place updated with the ellipse and cut parameters.
void addRectInfo(const Place2d::CloudT& points,
                 const double connection_ellipse_scale_factor,
                 Place2d& place);

/// @brief Compute the boundary of a place and store it on the place.
void addBoundaryInfo(const Place2d::CloudT& points, Place2d& place);

/// @brief Compute the boundary of a place and store it on node attributes.
void addBoundaryInfo(const Place2d::CloudT& points, Place2dNodeAttributes& attrs);

/// @brief Split a place into two along its cut plane.
/// @param points Vertex positions referenced by the place.
/// @param place Place to divide.
/// @param connection_ellipse_scale_factor Scale applied to the children's ellipses.
/// @return The two resulting places; children may be empty for a degenerate cut.
std::pair<Place2d, Place2d> splitPlace(const Place2d::CloudT& points,
                                       const Place2d& place,
                                       const double connection_ellipse_scale_factor);

/// @brief Recursively split a set of places until each meets the size limits.
/// @param cloud Vertex positions referenced by the places.
/// @param initial_places Places to decompose.
/// @param min_size Target maximum extent before a place stops splitting.
/// @param min_points Minimum points a place must keep to be split further.
/// @param connection_ellipse_scale_factor Scale applied to the resulting ellipses.
/// @return The decomposed places.
std::vector<Place2d> decomposePlaces(const Place2d::CloudT& cloud,
                                     const std::vector<Place2d>& initial_places,
                                     double min_size,
                                     size_t min_points,
                                     const double connection_ellipse_scale_factor);

/// @brief Recursively split a single place until it meets the size limits.
/// @param cloud_pts Vertex positions referenced by the place.
/// @param place Place to decompose.
/// @param min_size Target maximum extent before splitting stops.
/// @param min_points Minimum points a place must keep to be split further.
/// @param connection_ellipse_scale_factor Scale applied to the resulting ellipses.
/// @return The decomposed places.
std::vector<Place2d> decomposePlace(const Place2d::CloudT& cloud_pts,
                                    const Place2d& place,
                                    const double min_size,
                                    const size_t min_points,
                                    const double connection_ellipse_scale_factor);

/// @brief Decide whether two places overlap enough to be connected by an edge.
/// @param attrs1 Attributes of the first place.
/// @param attrs2 Attributes of the second place.
/// @param place_overlap_threshold Minimum ellipse overlap required.
/// @param place_max_neighbor_z_diff Maximum allowed height difference (m).
/// @param edge_attrs Populated with the edge weight when a connection is made.
/// @return True if the places should be connected.
bool shouldAddPlaceConnection(const Place2dNodeAttributes& attrs1,
                              const Place2dNodeAttributes& attrs2,
                              const double place_overlap_threshold,
                              const double place_max_neighbor_z_diff,
                              EdgeAttributes& edge_attrs);

/// @brief Decide whether two places overlap enough to be connected by an edge.
/// @param p1 First place.
/// @param p2 Second place.
/// @param place_overlap_threshold Minimum ellipse overlap required.
/// @param place_max_neighbor_z_diff Maximum allowed height difference (m).
/// @param weight Populated with the edge weight when a connection is made.
/// @return True if the places should be connected.
bool shouldAddPlaceConnection(const Place2d& p1,
                              const Place2d& p2,
                              const double place_overlap_threshold,
                              const double place_max_neighbor_z_diff,
                              double& weight);

}  // namespace hydra

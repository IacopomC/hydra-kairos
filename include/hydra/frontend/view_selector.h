#pragma once
#include <spark_dsg/node_attributes.h>

#include "hydra/openset/openset_types.h"

/**
 * @file view_selector.h
 * @brief A single sensor observation paired with a feature vector, plus the
 *        strategy interface for choosing which observation feeds a node's
 *        semantic feature.
 */

namespace hydra {

class Sensor;

/// @brief One feature vector observed from a known sensor pose at a given time.
struct FeatureView {
  using Ptr = std::unique_ptr<FeatureView>;

  /// @brief Construct a view from its timestamp, sensor pose, feature, and sensor model.
  /// @param timestamp_ns Observation time in nanoseconds.
  /// @param sensor_T_world Transform mapping world points into the sensor frame.
  /// @param feature Feature vector extracted for this view.
  /// @param sensor Sensor model used for frustum tests; must be non-null.
  FeatureView(uint64_t timestamp_ns,
              const Eigen::Isometry3d& sensor_T_world,
              const FeatureVector& feature,
              const Sensor* sensor);

  const uint64_t timestamp_ns;            ///< Observation time in nanoseconds.
  const Eigen::Isometry3d sensor_T_world; ///< Transform from world frame to sensor frame.
  const FeatureVector feature;            ///< Feature vector extracted for this view.

  /// @brief Access the sensor model backing this view.
  const Sensor& sensor() const;

  /// @brief Test whether a world point falls inside this view's frustum.
  /// @param point_w Query point in the world frame.
  /// @param point_s Optional output for the point expressed in the sensor frame.
  /// @return True if the point lies within the sensor's view frustum.
  bool pointInView(const Eigen::Vector3d& point_w,
                   Eigen::Vector3d* point_s = nullptr) const;

 private:
  const Sensor* const sensor_;  ///< Sensor model used for frustum tests.
};

/// @brief Strategy that picks (or fuses) a feature for a node from candidate views.
struct ViewSelector {
  using FeatureList = std::list<FeatureView::Ptr>;  ///< List of candidate views.

  virtual ~ViewSelector() = default;

  /// @brief Assign the node's semantic feature from the candidate views.
  /// @param views Views visible to the node.
  /// @param attrs Node attributes whose semantic feature is updated in place.
  virtual void selectFeature(const FeatureList& views,
                             spark_dsg::SemanticNodeAttributes& attrs) const = 0;
};

}  // namespace hydra

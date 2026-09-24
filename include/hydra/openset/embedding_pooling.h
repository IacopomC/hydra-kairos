#pragma once
#include <config_utilities/factory.h>

#include <Eigen/Dense>

#include "hydra/openset/openset_types.h"

/**
 * @file embedding_pooling.h
 * @brief Strategies for reducing a group of embeddings into a single embedding.
 *        Defines the pooling interface and concrete mean, weighted-mean, and
 *        max-weight implementations selectable through the configuration factory.
 */

namespace hydra {

/// @brief Interface that collapses a group of embeddings into one embedding.
struct FeaturePooling {
  using Ptr = std::unique_ptr<FeaturePooling>;
  using WeightVector = Eigen::VectorXf;  ///< Per-embedding weights, one entry per matrix column.
  virtual ~FeaturePooling() = default;

  /// @brief Combine the columns of @p features into a single embedding.
  /// @param features Embeddings packed as matrix columns.
  /// @param weights Per-column weights; used or ignored depending on the strategy.
  /// @return Pooled embedding, or an empty vector when there is nothing to pool.
  virtual FeatureVector pool(const FeatureGroup& features,
                             const WeightVector& weights) const = 0;
};

/// @brief Pools by taking the unweighted row-wise mean of the embeddings.
struct MeanPooling : FeaturePooling {
  struct Config {};
  explicit MeanPooling(const Config& = {}) {}
  /// @brief Return the mean embedding; weights are ignored.
  FeatureVector pool(const FeatureGroup& features,
                     const WeightVector& weights) const override;

 private:
  inline static const auto registration_ =
      config::RegistrationWithConfig<FeaturePooling, MeanPooling, Config>("mean");
};

void declare_config(MeanPooling::Config&);

/// @brief Pools by summing weighted embeddings, then dividing by the count.
struct WeightedMeanPooling : FeaturePooling {
  struct Config {};
  explicit WeightedMeanPooling(const Config& = {}) {}
  /// @brief Return the weighted sum divided by the number of embeddings.
  /// @note Yields an empty vector when the weight count does not match the columns.
  FeatureVector pool(const FeatureGroup& features,
                     const WeightVector& weights) const override;

 private:
  inline static const auto registration_ =
      config::RegistrationWithConfig<FeaturePooling, WeightedMeanPooling, Config>(
          "weighted_mean");
};

void declare_config(WeightedMeanPooling::Config&);

/// @brief Pools by selecting the embedding with the largest weight.
struct MaxPooling : FeaturePooling {
  struct Config {};
  explicit MaxPooling(const Config& = {}) {}
  /// @brief Return the column whose weight is highest.
  /// @note Yields an empty vector when the weight count does not match the columns.
  FeatureVector pool(const FeatureGroup& features,
                     const WeightVector& weights) const override;

 private:
  inline static const auto registration_ =
      config::RegistrationWithConfig<FeaturePooling, MaxPooling, Config>("max");
};

void declare_config(MaxPooling::Config&);

}  // namespace hydra

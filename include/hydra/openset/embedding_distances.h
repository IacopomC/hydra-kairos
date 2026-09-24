#pragma once
#include <config_utilities/config.h>
#include <config_utilities/validation.h>
#include <config_utilities/virtual_config.h>

#include <Eigen/Dense>

#include "hydra/openset/embedding_group.h"

/**
 * @file embedding_distances.h
 * @brief Distance and similarity metrics for comparing semantic embeddings.
 *        Defines a common metric interface and concrete cosine, L1, L2, and
 *        softmax-style scoring implementations registered with the config factory.
 */

namespace hydra {

/// @brief Interface pairing a dissimilarity distance with a similarity score.
struct EmbeddingDistance {
  virtual ~EmbeddingDistance() = default;

  /// @brief Call operator returning the similarity score of the two embeddings.
  inline float operator()(const FeatureVector& lhs, const FeatureVector& rhs) const {
    return score(lhs, rhs);
  }

  /// @brief Dissimilarity between two embeddings; larger means farther apart.
  virtual float dist(const FeatureVector& lhs, const FeatureVector& rhs) const = 0;

  /// @brief Similarity between two embeddings; larger means more alike.
  virtual float score(const FeatureVector& lhs, const FeatureVector& rhs) const = 0;
};

/// @brief Cosine metric; score is cosine similarity, distance is one minus it.
struct CosineDistance : EmbeddingDistance {
  struct Config {
    float tolerance = 1.0e-9f;  ///< Floor on the norm product to avoid dividing by zero.
  };

  explicit CosineDistance() : CosineDistance(Config()) {}

  explicit CosineDistance(const Config& config) : config(config::checkValid(config)) {}

  /// @brief One minus the cosine similarity of the two embeddings.
  float dist(const FeatureVector& lhs, const FeatureVector& rhs) const override;

  /// @brief Cosine similarity, the dot product over the clamped norm product.
  float score(const FeatureVector& lhs, const FeatureVector& rhs) const override;

  const Config config;  ///< Validated configuration.

 private:
  inline static const auto registration_ =
      config::RegistrationWithConfig<EmbeddingDistance, CosineDistance, Config>(
          "cosine");
};

inline void declare_config(CosineDistance::Config& config) {
  config::name("CosineDistance::Config");
  config::field(config.tolerance, "tolerance");
  config::check(config.tolerance, config::GT, 0.0, "tolerance");
}

/// @brief L1 metric on the normalized embeddings; score is one minus half the distance.
struct L1Norm : public EmbeddingDistance {
  struct Config {};

  explicit L1Norm(const Config& = {}) {}

  /// @brief L1 norm of the difference between the normalized embeddings.
  float dist(const FeatureVector& lhs, const FeatureVector& rhs) const override;

  /// @brief Similarity mapping the distance range onto a unit interval.
  float score(const FeatureVector& lhs, const FeatureVector& rhs) const override;

 private:
  inline static const auto registration_ =
      config::RegistrationWithConfig<EmbeddingDistance, L1Norm, Config>("l1");
};

inline void declare_config(L1Norm::Config&) { config::name("L1Norm::Config"); }

/// @brief L2 metric on the normalized embeddings; score is one minus half the distance.
struct L2Norm : public EmbeddingDistance {
  struct Config {};

  explicit L2Norm(const Config& = {}) {}

  /// @brief L2 norm of the difference between the normalized embeddings.
  float dist(const FeatureVector& lhs, const FeatureVector& rhs) const override;

  /// @brief Similarity mapping the distance range onto a unit interval.
  float score(const FeatureVector& lhs, const FeatureVector& rhs) const override;

 private:
  inline static const auto registration_ =
      config::RegistrationWithConfig<EmbeddingDistance, L2Norm, Config>("l2");
};

inline void declare_config(L2Norm::Config&) { config::name("L2Norm::Config"); }

/// @brief Softmax-style score weighing the query against a set of reference embeddings.
struct LerfScore : public EmbeddingDistance {
  struct Config {
    config::VirtualConfig<EmbeddingGroup> cannonical_features;  ///< Reference embeddings the query competes against.
    float tolerance = 1.0e-9f;                                  ///< Floor passed to the underlying cosine metric.
  };

  explicit LerfScore(const Config& config);

  /// @brief One minus the score; a convenience dissimilarity, not a true metric.
  float dist(const FeatureVector& lhs, const FeatureVector& rhs) const override;

  /// @brief Smallest softmax weight of the query against each reference embedding.
  float score(const FeatureVector& lhs, const FeatureVector& rhs) const override;

  const Config config;  ///< Validated configuration.

 private:
  std::unique_ptr<EmbeddingGroup> cannonical_;  ///< Reference embeddings built from the configuration.

  inline static const auto registration_ =
      config::RegistrationWithConfig<EmbeddingDistance, LerfScore, Config>("lerf");
};

void declare_config(LerfScore::Config& config);

}  // namespace hydra

#pragma once
#include <Eigen/Dense>
#include <memory>
#include <string>
#include <vector>

#include "hydra/openset/openset_types.h"

/**
 * @file embedding_group.h
 * @brief A named collection of semantic embeddings that can be compared against
 *        a query embedding. Provides per-entry distances and scores under a
 *        chosen distance metric, plus a lookup of the best-scoring entry.
 */

namespace hydra {

struct EmbeddingDistance;

/// @brief Collection of named embeddings compared against a query embedding.
struct EmbeddingGroup {
  using Ptr = std::shared_ptr<EmbeddingGroup>;

  /// @brief Highest score found in the group together with its entry index.
  struct ScoreResult {
    float score = std::numeric_limits<float>::lowest();  ///< Best score; initialized to the lowest float.
    size_t index = 0;                                    ///< Index of the best-scoring embedding.
  };

  virtual ~EmbeddingGroup();

  /// @brief True when the group holds no embeddings.
  bool empty() const;
  /// @brief Boolean conversion that is true when the group is non-empty.
  operator bool() const { return !empty(); }
  /// @brief Number of embeddings in the group.
  size_t size() const { return embeddings.size(); }

  /// @brief Distance from the query to every stored embedding.
  /// @param dist Distance metric to apply.
  /// @param embedding Query embedding.
  /// @return Vector of distances, one per stored embedding.
  Eigen::VectorXf getDistances(const EmbeddingDistance& dist,
                               const FeatureVector& embedding) const;

  /// @brief Similarity score from the query to every stored embedding.
  /// @param dist Distance metric to apply.
  /// @param embedding Query embedding.
  /// @return Vector of scores, one per stored embedding.
  Eigen::VectorXf getScores(const EmbeddingDistance& dist,
                            const FeatureVector& embedding) const;

  /// @brief Highest-scoring stored embedding for the query.
  /// @param dist Distance metric to apply.
  /// @param embedding Query embedding.
  /// @return Best score and the index of the embedding that produced it.
  ScoreResult getBestScore(const EmbeddingDistance& dist,
                           const FeatureVector& embedding) const;

  std::vector<FeatureVector> embeddings;  ///< Stored embeddings.
  std::vector<std::string> names;         ///< Optional labels paired with the embeddings.
};

}  // namespace hydra

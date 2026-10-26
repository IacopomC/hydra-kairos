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
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
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
#include <gtest/gtest.h>
#include <hydra/backend/merge_tracker.h>

#include "hydra_test/shared_dsg_fixture.h"

namespace hydra {

TEST(MergeTracker, KeepsMergeHistoryAcrossCalls) {
  auto dsg = test::makeSharedDsg();
  auto& graph = *dsg->graph;
  for (const auto id : {"p0"_id, "p1"_id, "p2"_id}) {
    graph.emplaceNode(DsgLayers::PLACES, id, std::make_unique<PlaceNodeAttributes>());
  }
  const auto unmerged = graph.clone();

  MergeTracker tracker;
  EXPECT_EQ(1u, tracker.applyMerges(*unmerged, {{"p1"_id, "p0"_id}}, *dsg));
  EXPECT_FALSE(graph.hasNode("p1"_id));
  EXPECT_EQ(dsg->merges.at("p1"_id), "p0"_id);

  // a later pass must not forget earlier merges, even though the absorbed node
  // is no longer in the graph
  EXPECT_EQ(0u, tracker.applyMerges(*unmerged, {}, *dsg));
  ASSERT_EQ(1u, dsg->merges.count("p1"_id));
  EXPECT_EQ(dsg->merges.at("p1"_id), "p0"_id);

  // a proposal involving the absorbed node is remapped onto its target, and the
  // absorbed node follows the target into the new merge
  EXPECT_EQ(1u, tracker.applyMerges(*unmerged, {{"p1"_id, "p2"_id}}, *dsg));
  EXPECT_FALSE(graph.hasNode("p0"_id));
  EXPECT_EQ(dsg->merges.at("p0"_id), "p2"_id);
  EXPECT_EQ(dsg->merges.at("p1"_id), "p2"_id);
}

TEST(MergeTracker, DropsMergesOntoRemovedNodes) {
  auto dsg = test::makeSharedDsg();
  auto& graph = *dsg->graph;
  for (const auto id : {"p0"_id, "p1"_id}) {
    graph.emplaceNode(DsgLayers::PLACES, id, std::make_unique<PlaceNodeAttributes>());
  }
  const auto unmerged = graph.clone();

  MergeTracker tracker;
  EXPECT_EQ(1u, tracker.applyMerges(*unmerged, {{"p1"_id, "p0"_id}}, *dsg));
  graph.removeNode("p0"_id);

  // the target is gone, so its merges are dropped and stale proposals are skipped
  EXPECT_EQ(0u, tracker.applyMerges(*unmerged, {{"p1"_id, "p0"_id}}, *dsg));
  EXPECT_TRUE(dsg->merges.empty());
}

}  // namespace hydra

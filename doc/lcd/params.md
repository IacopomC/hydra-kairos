# Loop Closure Detection (LCD) Parameters

This document describes the YAML parameters that control LCD in hydra-kairos.

## Files involved

- `config/lcd/default.yaml` — the LCD config that gets merged in when
  `pipeline.py` is launched with `enable_lcd=True`.
- `config/lcd/default_gnn.yaml`, `config/lcd/uhumans2*.yaml` — variants for
  GNN-based descriptors / uHumans2.
- `config/datasets/<name>.yaml` — dataset-level overrides. Anything you put
  here under the same path overrides the LCD default. The shipped dataset
  configs all inherit `default.yaml` unchanged.

## How the YAML maps to C++

The python entry point loads the dataset YAML, then merges
`config/lcd/<lcd_config>.yaml` in over it. The merged tree is bound to
`LoopClosureConfig` (see `include/hydra/loop_closure/loop_closure_config.h`),
which contains:

- a few top-level fields (horizons, visualizer namespace), and
- a nested `lcd:` block bound to `LcdDetectorConfig`
  (`include/hydra/loop_closure/detector.h`).

The `LcdDetectorConfig` contains, in turn, sub-structs for the agent /
objects / places layers, the TEASER solver, and (optionally) a GNN block.

---

## 1. Top-level horizons

Set at the root of the LCD YAML.

| Parameter | Type | Default | What it controls |
|---|---|---|---|
| `lcd_agent_horizon_s` | `double` | `1.5` (yaml: `3.5`) | Time window over which agent-layer descriptors are accumulated. Bigger = smoother descriptors, but laggier matches. |
| `descriptor_creation_horizon_m` | `double` | `10.0` (yaml: `15.0`) | Minimum agent-to-place distance (metres) before a place becomes eligible for LCD descriptor caching. Places inside this radius are kept pending and rechecked on later ticks. Bigger = wait longer before caching = more stable but more memory in the pending queue. |
| `lcd_visualizer_ns` | `string` | `/dsg/lcd_visualizer` | ROS namespace for the LCD visualizer (only meaningful in the ROS wrapper). |

**Tuning intuition.** `descriptor_creation_horizon_m` is a spatial distance
from the agent. A place is held back from LCD caching while the agent is closer than
this; once the agent is farther, the place is released into the cached set
on the next pass. Pick it according to how much the
robot moves around in your scenario — small space → small horizon is fine.

**Interaction with the frontend's active-place radius.** The frontend keeps
places `is_active=true` while they are within
`frontend.surface_places.active_place_radius_m` (default `16.0`) and
flips them inactive once the agent is farther. LCD only releases a place
once it is **both** beyond `descriptor_creation_horizon_m` **and** inactive.
When `descriptor_creation_horizon_m <<
active_place_radius_m` (e.g. `3` vs `16`), there is a wide band where LCD
considers a place eligible but the frontend has reactivated it on a
revisit; the LCD module skips such places and rechecks on a later tick.
Two practical knobs if you see "Skipping reactivated place ..." piling up
in VLOG(5):
- raise `descriptor_creation_horizon_m` toward `active_place_radius_m`, or
- lower `frontend.surface_places.active_place_radius_m` toward
  `descriptor_creation_horizon_m`.

Either keeps the queue draining at the same rate places enter it.

Defined: `include/hydra/loop_closure/loop_closure_config.h`,
declared in `src/loop_closure/loop_closure_config.cpp`.

---

## 2. Subgraph extraction (per layer)

Under `lcd:`, two `SubgraphConfig`s govern how many nodes around a candidate
root are pulled into a place / object descriptor:

```yaml
lcd:
  object_extraction: {fixed_radius: true, max_radius_m: 13.0}
  places_extraction: {fixed_radius: true, max_radius_m: 13.0}
```

| Field | Type | Notes |
|---|---|---|
| `fixed_radius` | `bool` | If `true`, take all nodes within `max_radius_m`. If `false`, grow the radius from `min_radius_m` until at least `min_nodes` are included or `max_radius_m` is hit. |
| `max_radius_m` | `double` | Hard cap on subgraph radius. Required. |
| `min_radius_m` | `double` | Floor used when `fixed_radius=false`. |
| `min_nodes` | `size_t` | Adaptive-mode target node count. |

**Tuning intuition.** This is the right knob to shrink if you want
descriptors that describe a small region (e.g. one corner of a hall). In `default_gnn.yaml` these become
`fixed_radius=false, min_radius_m=3, max_radius_m=5, min_nodes=10`, which is
the form to copy for tight descriptors.

Defined: `include/hydra/loop_closure/subgraph_extraction.h`.

---

## 3. Per-layer matching

For each of `agent`, `objects`, and `places` there is a `search:` block
(plus, for objects/places, a `registration:` block).

### 3.1 `search:` — `DescriptorMatchConfig`

```yaml
agent:
  min_time_separation_s: 25.0
  min_score: 0.040
  min_registration_score: 0.01
  max_registration_matches: 2
  min_score_ratio: 0.95
  min_match_separation_m: 0.0
  type: L1
```

| Field | Type | Default | Effect |
|---|---|---|---|
| `min_time_separation_s` | `double` | `0.0` | Reject candidates younger than this many seconds vs. the query. Prevents matching ourselves. |
| `min_score` | `float` | `1.0` | Lower bound on the descriptor distance (L1) or similarity (COSINE) for a candidate to be considered. The scale depends on `type`. |
| `min_registration_score` | `float` | `1.0` | Stronger threshold the candidate must clear to be passed to registration. Bypassed/overridden in some code paths — see comment in `default.yaml:14`. |
| `max_registration_matches` | `size_t` | `5` | At most this many candidates are forwarded to the (expensive) registration solver. |
| `min_score_ratio` | `double` | `0.7` | Lowe-style ratio test between best and 2nd-best candidates. |
| `min_match_separation_m` | `double` | `5.0` | Spatial spacing required between accepted matches; prevents a cluster of nearby candidates flooding registration. |
| `type` | enum `L1`/`COSINE` | `L1` | Distance metric used for `min_score` etc. |

**Per-layer notes.**
- `agent` matches use embeddings of recent agent (camera) trajectories.
  `enable_agent_registration: true` (one level above) routes them through
  the registration solver.
- `objects` and `places` search blocks have the same shape but different
  thresholds reflecting how much each descriptor type discriminates.

Defined: `include/hydra/loop_closure/descriptor_matching.h`.

### 3.2 `registration:` — `LayerRegistrationConfig`

Only on `objects` and `places`:

```yaml
objects:
  registration:
    min_correspondences: 5
    min_inliers: 5
    log_registration_problem: false
    registration_output_path: ''
    recreate_subgraph: false
```

| Field | Type | Default | Effect |
|---|---|---|---|
| `min_correspondences` | `size_t` | `5` | Minimum point pairs required to even attempt the solve. |
| `min_inliers` | `size_t` | `5` | Minimum inliers TEASER must agree on before the candidate is accepted. |
| `max_same_nodes` | `size_t` | `2` | Cap on times a single node can be reused in correspondences. |
| `recreate_subgraph` | `bool` | `false` | When `true`, re-extracts the subgraph using `subgraph_extraction:` after a candidate match (used by GNN configs). |
| `log_registration_problem` | `bool` | `false` | Dumps the registration problem to disk for offline replay. |
| `registration_output_path` | `string` | `""` | Where the dump goes when logging is on. |

Defined: `include/hydra/loop_closure/registration.h`.

---

## 4. Detector-level toggles

Sit directly under `lcd:`.

| Parameter | Type | Default | Effect |
|---|---|---|---|
| `enable_agent_registration` | `bool` | `true` | If `false`, agent descriptors are still scored but no level-0 registration is attempted. The level-0 solver depends on the wrapper: `hydra-kairos-ros` plugs in `DsgAgentSolver` (RANSAC + P3P via Kimera-VIO's `frame_registration` ROS service); the offline python bindings plug in `BowAgentSolver` (BoW similarity + dedup; see §9). TEASER runs only at objects/places. |
| `num_semantic_classes` | `size_t` | `20` | Number of one-hot bins for object descriptors. Should match the labelspace; the python pipeline overrides this from `GlobalInfo`, so changes here are usually ignored. |
| `place_histogram_config` | struct `{min,max,bins}` | `{0.5, 2.5, 30}` | Histogram bounds (in metres) and bin count for the GVD-distance histogram used as the place descriptor. |

Defined: `include/hydra/loop_closure/detector.h`.

---

## 5. TEASER solver

`lcd.teaser:` binds to TEASER++'s `RobustRegistrationSolver::Params`. Most
defaults are fine. The fields you may want to touch:

| Field | Default | When to change |
|---|---|---|
| `noise_bound` | `0.100` | Increase if your point/place positions are noisier than ~10 cm. |
| `cbar2` | `1.0` | Outlier truncation; larger = more permissive. |
| `rotation_max_iterations` | `100` | Cap iterations to bound runtime. |
| `kcore_heuristic_threshold` | `0.5` | Max-clique heuristic threshold; lower = stricter. |
| `inlier_selection_mode` | `PMC_EXACT` | `PMC_EXACT` is exact but slow; `PMC_HEU` / `KCORE_HEU` / `NONE` trade accuracy for speed. |
| `max_clique_time_limit` | `3600` (s) | Hard wall-clock limit on max-clique search. |

**Memory cost (important).** TEASER's `computeTIMs` + scale solver allocate **O(C²)**
in the number of correspondences `C ≈ (#query-subgraph nodes) × (#match-subgraph nodes)`.
For the **places** layer this scales with **L3 density**: a dense L3 inside the
`places_extraction` radius yields thousands of correspondences → **tens of GB transient
spikes** (observed ~30 GB at ~190 places / 5 m). The **objects** layer is sparse and
cheap to register; registering the places layer
is safe only with a **sparse** L3 and/or a small `places_extraction.max_radius_m`.
If your scene is feature-poor (agent BoW + objects don't fire) and you need
places-LCD anyway, feed loop closures from an external VPR (e.g.
MIT-SPARK/Ouroboros) through `external_loop_closure_queue`; this keeps TEASER off
the dense place graph.

Declared: `src/loop_closure/loop_closure_config.cpp`.

---

## 6. Cross-cutting: how a single match flows through

The order in which fields gate a candidate match:

1. **Subgraph extraction** (`object_extraction` / `places_extraction`):
   build a query descriptor from a region of radius `max_radius_m` around
   the current root.
2. **Descriptor scoring** (`<layer>.search.min_score`,
   `min_score_ratio`, `min_time_separation_s`,
   `min_match_separation_m`): pick top-k candidates.
3. **Registration gating** (`<layer>.search.min_registration_score`,
   `max_registration_matches`): keep only the top few for solving.
4. **TEASER** (`teaser.*`, `<layer>.registration.min_correspondences`,
   `min_inliers`): final 6-DoF transform; accepted only if it passes
   `min_inliers`.
5. The accepted match is handed to the backend, which adds a loop-closure
   factor and triggers a deformation graph update.

If you don't see any matches at all, walk the chain top-down: first verify
descriptors are being created (step 1), then loosen `min_score` (step 2),
then `min_registration_score` (step 3), and only last touch TEASER.

---

## 7. Tuning recipes

### Single hall / open indoor space (TBD-style)

- `lcd_agent_horizon_s: 3.5` — keep, agent layer is fine.
- `descriptor_creation_horizon_m: 10–15` — keep close to the default;
  shrinking it below the loop-closure travel distance starves the
  descriptor.
- `object_extraction.max_radius_m`, `places_extraction.max_radius_m: 5–8`
  — modestly smaller than default (`13`) so descriptors discriminate
  *within* the hall instead of describing the whole hall every time.
- `min_match_separation_m: 2–3` — default `5` may exclude legitimate
  re-visits of the same corner.
- Leave thresholds (`min_score`, `min_registration_score`) at defaults
  initially; loosen one rung at a time if no candidates clear.

### Sparse semantics (few object classes)

- Reduce `num_semantic_classes` to the actual class count (or rely on the
  python override).
- `objects.search.min_score`: drop slightly; sparser descriptors are more
  similar to one another so the absolute score floor matters more than the
  ratio test.

### When tuning the `map_window` instead of LCD

The active-window radius (`tbd.yaml: map_window.max_radius_m`) belongs to the
active window, and it affects LCD heavily: it controls when blocks (and hence
places) are archived. Setting it too small (e.g. 5 m in a hall whose camera
range is also 5 m) causes constant archiving churn, which feeds into the
deformation graph that LCD updates. Keep it ≥ ~2× the camera FOV radius.

---

## 8. Quick reference

| Knob | YAML path | Likely first thing to try |
|---|---|---|
| Match too few | `lcd.<layer>.search.min_score` | Lower by 0.05 |
| Match too eagerly | same | Raise by 0.05 |
| Drift at registration | `lcd.<layer>.registration.min_inliers` | Raise to 7–10 |
| Slow registration | `lcd.teaser.inlier_selection_mode` | Switch to `PMC_HEU` |
| Descriptors look uniform | `<layer>_extraction.max_radius_m` | Shrink |
| Self-matching | `<layer>.search.min_time_separation_s` | Raise |

---

## 9. Offline (python) LCD: agent-layer with BoW

> **Summary.** Agent-layer LCD fires offline. On top of the offline BoW
> producer (§9.3), a `BowAgentSolver`
> (`include/hydra/loop_closure/bow_agent_solver.h`) scores BoW
> similarity between agents and short-circuits the cascade with an
> odometry-derived relative pose, with cross-call dedup so the same anchor
> triggers the backend once per revisit. Places-LCD runs only as the
> fallback when the agent layer returns invalid (§9.4).

### 9.1 What the agent layer does in stock Hydra

In `hydra-kairos-ros`, two things happen at the agent layer:

1. **BoW descriptor matching.** Each agent node has `dbow_ids` /
   `dbow_values` populated from a Kimera-VIO BoW stream
   (`hydra-kairos-ros/hydra_ros/src/utils/bow_subscriber.cpp`). The
   detector scores BoW similarity to find candidate revisits.
2. **6-DoF visual verification.** `lcd::DsgAgentSolver`
   (`hydra-kairos-ros/hydra_ros/src/loop_closure/ros_lcd_registration.cpp`)
   calls a ROS service named `frame_registration`, served by
   **Kimera-VIO-ROS**, that takes `(query_robot, match_robot, query_frame_id,
   match_frame_id)` and returns a transform.

The python pipeline provides path 1 through the offline BoW producer (see
§9.3) and path 2 through `BowAgentSolver` (§9.4), which returns an
odometry-derived relative pose (§9.6).

### 9.2 Wiring overview

```
                          (offline)
hydra_python.bow_producer.BowProducerCallback
        │  ORB on packet.color
        │  DBoW3 transform
        │  push BowQuery onto bow_queue
        ▼
PipelineQueues::bow_queue ────▶ GraphBuilder::assignBowVectors
                                 stamps dbow_ids / dbow_values onto
                                 AgentNodeAttributes
                                 ▼
                          LcdDetector descriptor matching at level 0
                                 ▼
                          registerAndVerify cascade:
                            agent: BowAgentSolver (offline)
                              - L1 BoW similarity vs match anchor
                              - cross-call dedup against same anchor
                              - returns odometry-derived match_T_query
                              -> if valid, cascade short-circuits
                            objects (TEASER, fallback)
                            places (TEASER, fallback)
                                 ▼
                          first valid registration wins
```

`enable_agent_registration` is forced `true` in `pipeline.py`. The
python binding registers a `BowAgentSolver` at level 0.

### 9.3 Producer setup

See `doc/non_ros_usage.md §11` for the full recipe. In short:

```bash
bash scripts/fetch_orb_vocab.sh                   # one-time, ~50 MB
python -m hydra_python run <scene> -c <config> \
    --bow-vocab ~/.cache/hydra-kairos/orb_vocab.txt \
    -o <out>
```

The producer:

- Mirrors `PoseGraphFromOdom::update` keyframe selection in Python so the
  `pose_id` it emits matches the C++ agent key.
- Pushes per-keyframe via the C++ `BowProducer` class
  (`python/bindings/src/python_bow.cpp`), which runs `cv::ORB::detectAndCompute`
  + `DBoW3::Vocabulary::transform` and pushes a
  `pose_graph_tools::BowQuery` onto `PipelineQueues::bow_queue`.
- Logs `[Hydra Frontend] assigned N bow vectors of M original` from
  `assignBowVectors` — `M` drains to ~0 each step. If `M` grows
  unboundedly, pose_ids are misaligned (see §9.5 troubleshooting).

### 9.4 BowAgentSolver

`BowAgentSolver::solve` (`src/loop_closure/bow_agent_solver.cpp`):

1. Read `dbow_ids` / `dbow_values` off both the query and the candidate
   match agent's `AgentNodeAttributes`. If either is empty, return invalid.
2. Compute DBoW2-style L1 similarity:
   `score = 1 - 0.5 * Σ |v[i] - w[i]|`, range `[0, 1]`. Identical vectors
   score 1.0; disjoint L1-normalised vectors score 0.0. For ORB+ORBvoc.txt
   on indoor scenes, scores cluster around 0.05-0.20 for unrelated
   keyframes and 0.30-0.60 for true revisits.
3. If `score < bow_agent.min_score` (default 0.20), return invalid.
4. **Cross-call dedup.** If we already fired an LC against this match
   anchor within the last `bow_agent.dedup_window_s` seconds (default
   30.0), return invalid.
5. Otherwise, return a valid solution whose `match_T_query` is the
   odometry-derived relative pose between the two agents.

**Odometry-derived pose.** The solver works from the BoW vectors alone,
so `match_T_query` is the relative pose odometry already gave. A false BoW
match therefore leaves the trajectory intact, and real drift is corrected
by the rest of the pose graph (§9.6).

**Net cascade behaviour.**

- Agent (BowAgentSolver): true revisit + score ≥ min_score + not
  dedup-suppressed → valid → cascade short-circuits, no places-LCD,
  no places TEASER.
- Agent: dedup-suppressed (same anchor recently) → invalid → cascade
  falls to objects → places. Places-LCD has its own
  `min_time_separation_s: 60` and `min_match_separation_m: 8` to gate
  these.
- Agent: BoW score below threshold → invalid → cascade falls to
  objects/places (the default cascade, place-LCD as last resort).

### 9.5 Tuning gotchas in this configuration

- **The agent layer has two separate `min_score`s**:
  - `lcd.agent.search.min_score` (DescriptorMatchConfig) — gates which
    candidates the *descriptor matcher* hands to the registration solver.
    Default `0.040`. Lower = more candidates surveyed; LC firing is
    decided by the solver threshold below.
  - `loop_closure.bow_agent.min_score` (BowAgentSolver::Config) — gates
    which of those candidates the *solver* accepts. Default `0.20`. This
    is the one that controls how many LCs actually fire. Tighten this
    first.
- **`bow_agent.dedup_window_s`.** Default `30` seconds. Larger window =
  more aggressive collapse of repeated matches against the same anchor;
  smaller window = more LCs per revisit. Keep larger than your typical
  revisit dwell time.
- **`places.search.min_registration_score`** gates the fallback. A value
  of `1.5` effectively disables place registration, which suits a setup
  where agent registration is the main source. Offline, with
  BowAgentSolver firing on the well-matched cases, places runs only
  when BoW fails; you want it loose enough to catch low-BoW revisits
  (e.g. very different lighting) but not so loose it fires on
  geometric look-alikes. The shipped default is 0.30.
- **Object descriptors are only useful when there are objects.** In
  scenarios with sparse or near-uniform object semantics (single hall,
  no furniture), `objects.search.min_score: 0.3` will yield zero
  candidates and the level will contribute nothing; places is the
  fallback for exactly this case. If you want
  objects to also contribute, lower `objects.search.min_score` to
  `0.2` and verify that the matches you get are not just noise.
- **Place subgraphs need to be discriminative.** `places_extraction.max_radius_m`
  controls how much of the GVD around a candidate root goes into the
  subgraph; if it's too large in a small space, every place looks the
  same. For TBD-style halls use the GNN-config value of 5 m; the default
  `13.0` is too wide for a single hall.
- **`assigned N bow vectors of M original` shows M growing forever.**
  The Python `KeyframeTracker` mirror has drifted from the C++
  `PoseGraphFromOdom::update` (e.g. someone changed the C++ keyframe rule
  without updating the Python mirror). Compare logs at the same frame:
  Python's `[bow_producer] pushed BoW for pose_id=N` versus C++'s
  `[Hydra Frontend] Adding agent ... (key: K)` (VLOG(5)). They should agree on N==K for every
  accepted keyframe.

### 9.6 Drift correction offline

BowAgentSolver returns an *odometry-derived* relative pose. Agent-LCD
fires and short-circuits the cascade, and the LC factor it adds confirms
odometry. Drift correction comes from the rest of the pose graph and from
places-LCD when it fires.

BowAgentSolver + places-LCD fallback is the offline LCD configuration used
for TBD.

## 10. References

- `include/hydra/loop_closure/loop_closure_config.h`
- `include/hydra/loop_closure/detector.h`
- `include/hydra/loop_closure/descriptor_matching.h`
- `include/hydra/loop_closure/registration.h`
- `include/hydra/loop_closure/subgraph_extraction.h`
- `src/loop_closure/loop_closure_config.cpp` (yaml↔struct binding)
- `python/src/hydra_python/pipeline.py` (how the LCD yaml gets merged)

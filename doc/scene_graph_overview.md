# Scene graph: how it's built (start here)

Orientation for the Hydra/Kairos pipeline — the things worth knowing *before* reading or
tuning the code. For turning the knobs, see [parameter_tuning.md](parameter_tuning.md).

## Repos / where the code lives
- **kairos** — the flow/temporal contribution (per-place dynamics over time). Python entry
  point `kairos run` (`kairos/python/src/kairos_python/commands/run.py`), class `KairosPipeline`.
- **hydra-kairos** — a fork of MIT Hydra; the actual **scene-graph construction lives here**:
  TSDF reconstruction, GVD/places, mesh, 2D places, rooms, backend optimisation.
- **hydra** — upstream MIT Hydra (unmodified reference).
- Two pybind modules, both linking the same `libhydra`: **`_hydra_bindings`** (data loaders,
  sensors) and **`_kairos_bindings`** (`KairosPipeline`). They each statically compiled their
  own glog/Sensor helpers, which caused load-time clashes until guarded — keep that in mind if
  you touch the bindings.

## The DSG layers (the numbering is NOT sequential)
| id | name | what it is | built by |
|----|------|-----------|----------|
| 2  | OBJECTS | clustered object detections | object segmenter |
| 3  | PLACES | freespace graph (GVD medial axis) | `GvdPlaceExtractor` |
| 4  | ROOMS | clusters of places | `UpdateRoomsFunctor` |
| 5  | BUILDINGS | clusters of rooms | |
| **20** | **MESH_PLACES** | 2D *surface* places, from the mesh | `Place2dSegmenter` |

`MESH_PLACES = 20` trips everyone up: it's the `place_2d` surface layer, not "the 20th layer."
L3 (freespace) and L20 (surface) are two *different* place representations built two different
ways — tuning one does nothing to the other.

## Data flow (with files)
```
depth + labels ─▶ TSDF (active_window) ─┬─▶ GVD (medial axis) ─▶ sparse graph ─▶ L3 PLACES
                                        │     src/places/gvd_integrator.cpp
                                        │     src/places/compression_graph_extractor.cpp
                                        │     driver: src/frontend/gvd_place_extractor.cpp
                                        └─▶ marching-cubes MESH ─▶ pgmo compress ─▶ cluster+split ─▶ L20
                                              src/reconstruction/mesh_integrator.cpp
                                              src/frontend/place_2d_segmenter.cpp
                                              src/frontend/place_2d_split_logic.cpp
backend update functors: src/backend/update_*_functor.cpp     rooms: src/rooms/room_finder.cpp
```
Key coupling to remember: the GVD needs the TSDF surface band to be **thicker than a voxel**
(`truncation_distance ≥ voxel_size`) or it's starved; the mesh vertex spacing is
`max(voxel_size, pgmo.mesh_resolution)` and can't go finer than the voxel.

## Node lifecycle: active vs archived (what `is_active` actually means)

The pipeline runs on a local **active window** (`map_window.max_radius_m`, ~5 m).
Nodes are not "alive only while observed" — they have a lifecycle worth knowing
before you reason about memory or revisits:

- **Active → archived (not deleted).** When a region leaves the active window its
  place nodes are **kept in the graph** and just flagged `is_active = false`
  (`src/frontend/graph_builder.cpp` sets this; the backend retains them). Archived
  means "already-mapped, outside the current window" — it is **still part of the
  map**, not dead/garbage.
- **Revisit reactivates the same node (by design).** On return, the GVD compressor
  matches re-observed voxels against **archived** clusters too
  (`src/places/compression_graph_extractor.cpp`, `findCluster`) and reuses the same
  node id, flipping `is_active` back to true (`gvd_place_extractor.cpp`;
  surface/2d places do the same in `place_2d_segmenter.cpp` / `mesh_segmenter.cpp`).
- **Optimization deforms archived nodes too.** Archived nodes are added to the
  deformation graph (`src/backend/mst_factors.cpp`) and, on a loop closure, the
  backend re-optimizes **all** nodes including archived (`update_places_functor.cpp`)
  — so old map regions stay geometrically consistent. It is *not* just the active
  nodes that move.
- **Where nodes are actually removed.** Frontend re-extraction when a place no
  longer corresponds to a valid GVD/cluster (`gvd_place_extractor.cpp`,
  `place_2d_segmenter.cpp`, `mesh_segmenter.cpp`) and backend node-**merging**
  (`merge_tracker.cpp`). There is **no** garbage collection of archived nodes by
  age or distance.
- **Dead keys (don't expect pruning):** `place_memory_radius_m`,
  `active_place_radius_m`, `mesh_active_window_m` are not read by any C++ code
  (a single comment mentions one) — they do **not** bound the archived set.

## The config system (this explains a whole class of "it does nothing" bugs)
- Configs use **config_utilities**: each YAML key binds to a C++ field by the **exact string**
  in `declare_config`'s `field(member, "name")`. **Unknown keys are silently dropped — no error,
  no warning.** A name mismatch or a typo means the value is ignored and the default is used.
  (This is exactly how `min_places_component_size` and `node_merge_distance_m` were silently
  ignored before being fixed — and why room-size knobs can look inert.)
- A dataset YAML (`config/datasets/*.yaml`) is an **override merged onto a base default** — you
  only list what differs from the defaults.
- A field of type `VirtualConfig<T>` picks a module by `type:`; `type: ""` selects **none**
  (e.g. disabling the TSDF downsampler so the GVD runs at native resolution).
- To learn what any key does: `grep -rn "the_key"` in `src/` and `include/` — it leads to the
  `field(...)` binding, the struct (its default), and the code that reads the value. If grep
  finds the key only in YAML and never in C++, the key is being silently ignored.

## Watch out for
- **Dead keys** (present in YAML, not bound in `place_2d_segmenter`): `min_final_place_size`,
  `place_memory_radius_m`, `active_place_radius_m`, `mesh_active_window_m`.
- **Frontend vs backend names differ**: the backend `Update2dPlacesFunctor` uses `min_size`/
  `min_points`/`connection_*` (under `backend.update_functors.surface_places`), not the
  frontend `pure_final_place_size`/`min_final_place_points`/`place_*`. Don't cross them.
- **Units**: most thresholds are meters, but `parent_l1_separation` is in *voxels* and the
  `min_*` filters are *counts*. (Full table in parameter_tuning.md.)

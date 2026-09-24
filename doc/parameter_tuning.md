# Tuning the place graph (L3 / L20)

How to control the density and connectivity of the scene-graph layers, and how to
re-tune for a new environment. The **mechanisms** here transfer between datasets;
the **specific numbers** (a TBD entrance hall at voxel 0.4) are examples — re-sweep
them for a new scene with the recipe at the end.

Layers: **L2** objects · **L3** freespace places (GVD) · **L4** rooms · **L20** surface
places (`place_2d`, from the mesh). New to the pipeline? Read
[scene_graph_overview.md](scene_graph_overview.md) first (architecture, layer map, config system).

## How each layer is built (1 line each)
- **L3:** TSDF → ESDF/GVD (medial axis) → `CompressionGraphExtractor` buckets the ridge
  into a grid of `compression_distance_m` and emits ~one node per occupied cell.
- **L20:** mesh vertices are Euclidean-clustered per label, each cluster recursively
  bisected; every leaf is a node. Edges = overlap of per-place inflated ellipses.
- **L4:** rooms cluster the L3 graph by dilation — so room count tracks L3 density.

## Know your units (the #1 source of confusion)
| in **meters** | in **voxels** | a **count** |
|---|---|---|
| `truncation_distance`, all `*_distance_m`, `*_m`, `cluster_tolerance`, `pure_final_place_size` | `parent_l1_separation` | `min_cluster_size`, `min_*_points`, `min_*_component_size`, `voxels_per_side` |

`parent_l1_separation` being in voxels is easy to miss (20 ≈ 8 m at voxel 0.4).

## Recipes by goal

**Freespace places (L3) too few / collapse to almost nothing**
1. `truncation_distance` must be **≥ voxel_size** (use ~1.5×). If it's smaller, the TSDF
   surface band is thinner than a voxel and the GVD is *starved* — this is the most common
   cause and the biggest lever. (Example: at voxel 0.4, 0.3→0.6 took L3 from 528 to 2729.)
2. Disable the GVD downsampler: `freespace_places.tsdf_interpolator: {type: ""}`. The default
   `{type: downsample, ratio: 2}` runs the GVD at *2× voxel*. (`ratio: 1` is rejected — must be
   even and >1; you disable the interpolator, not set ratio 1.)
3. `compression_distance_m` sets node spacing/count — best at **≈ voxel_size**. Going *below*
   the (effective) voxel fragments and *reduces* count. Dial it to hit a target.
4. Secondary (~±15% each): `gvd.min_distance_m`↑ (thicker seed band, keep < truncation),
   `voronoi_config.parent_l1_separation`↓, `voronoi_config.parent_cos_angle_separation`↑.
   Keep `voronoi_config.mode: L1_THEN_ANGLE` — `L1_DISTANCE` collapses L3 (the L1 test rarely fires).

**Surface places (L20) missing entirely**
`cluster_tolerance` must **exceed the mesh spacing** = `max(voxel_size, pgmo.mesh_resolution)`.
Below it, every vertex is its own cluster and all are dropped by `min_cluster_size`. (At voxel
0.4 spacing is 0.4, so the old 0.3 gave zero places; use ≥ 0.5.) Then scale `min_cluster_size`
down ~1/spacing² (≈16× from voxel 0.1→0.4).

**Surface places (L20) sparse or disconnected**
- More nodes: lower `pure_final_place_size` (the max final place size, in meters) — but it
  bottoms out at ~mesh spacing (too small → places fall below the 3-vertex hull and vanish).
  Example sweet spot at voxel 0.4: **0.5** (~398 nodes; 0.4→~225; 0.3→collapses).
- More edges / fewer disconnected components: raise **`connection_ellipse_scale_factor`**
  (default **1** = most disconnected; try 2.5–3.5). It enlarges each place's ellipse so neighbours
  overlap and link. Density and connectivity trade off (bigger ellipses also merge places), so
  pair a low `pure_final_place_size` with a high scale factor. `place_max_neighbor_z_diff` only
  matters on multi-level scenes (inert on a flat floor).

**Too many rooms (L4)**
It's a symptom of a dense L3, not a separate problem. To collapse toward 1 room, raise
`backend.update_functors.rooms.room_finder.{min_room_size, min_component_size}`.

## Gotchas
- **L3 nodes look grid-aligned** — by design: `compression_distance_m` quantises node positions
  to a grid. Not a bug.
- **Dead config keys** (silently ignored by `place_2d_segmenter`): `min_final_place_size`,
  `place_memory_radius_m`, `active_place_radius_m`, `mesh_active_window_m`. The backend
  `Update2dPlacesFunctor` uses *different* key names (`min_size`/`min_points`/`connection_*`)
  under `backend.update_functors.surface_places` — don't mix them up with the frontend ones.
- **node_merge_distance_m** must be **≤ ~compression/2**, or it over-merges and collapses the
  graph. (This key and `min_places_component_size` were silently ignored before a 2024 binding
  fix; if you're on an old checkout they may have no effect.)

## Recommended starting config (voxel 0.4, this scene)
```yaml
active_window:
  volumetric_map: {voxel_size: 0.4, truncation_distance: 0.6}   # truncation >= voxel
frontend:
  freespace_places:
    tsdf_interpolator: {type: ""}                  # native-res GVD
    graph: {compression_distance_m: 0.4, min_node_distance_m: 0.2, node_merge_distance_m: 0.2}
    gvd: {min_distance_m: 0.4, voronoi_config: {mode: L1_THEN_ANGLE, parent_l1_separation: 6, parent_cos_angle_separation: 0.4}}
  surface_places:
    cluster_tolerance: 0.6
    min_cluster_size: 3
    pure_final_place_size: 0.5
    connection_ellipse_scale_factor: 3.5
```

## Re-tuning for a new dataset (the method, not guesswork)
Don't grid-search (exponential). Use **one-at-a-time** sensitivity analysis:
1. Start from the config above; set `voxel_size`, then `truncation_distance = ~1.5 × voxel`.
2. Set `compression_distance_m ≈ voxel`, `cluster_tolerance > max(voxel, mesh_resolution)`.
3. Change **one** parameter, re-run, look at the node/edge counts; keep what helps.
4. Only after the single knobs are mapped, try a 2-D combo on the two that matter most
   (e.g. `pure_final_place_size` × `connection_ellipse_scale_factor` for L20).

## Evidence (TBD entrance hall, 1000 frames)
The measurements these recipes are based on — illustrative, not targets.

**Density vs voxel size, baseline params** (shows the resolution ceiling, and the L20 cliff at 0.3
where `cluster_tolerance 0.3` stops exceeding the mesh spacing):

| voxel | 0.1 | 0.2 | 0.3 | 0.4 |
|---|---|---|---|---|
| L3 | 847 | 230 | 209 | 79 |
| L20 | 633 | 389 | 0 | 0 |

**`truncation_distance` at voxel 0.4** (native GVD, compression 0.4) — the dominant L3 lever:
0.3 → **528**, 0.6 → **2729**, 0.8 → 2658.

**GVD secondary levers** (one-at-a-time from L3=2729): `gvd.min_distance` 0.2→0.4 **+455**,
`parent_l1_separation` 20→6 **+393**, `parent_cos_angle` 0.2→0.4 **+360**;
`voronoi.mode → L1_DISTANCE` **collapses to 65** (avoid).

**L20 nodes / edges / components**: default (`ces 1`) 305 / 55 / 254 (disconnected) →
`pure_final_place_size 0.5` × `connection_ellipse_scale_factor 3.5` → **398 / 1514 / 45**
(0.1 reference: 633 / 1458 / 5). The two knobs interact: more, well-populated places that are
also big enough to overlap once inflated.

## When to put places in the backend optimization (`add_places_to_deformation_graph`)

This flag decides whether the freespace-place layer (L3) is included as its own set
of variables in the backend's loop-closure optimization.

It does **not** decide whether the map is corrected after a loop closure. The
reconstructed geometry (mesh / voxels) and the trajectory are corrected either way —
the optimization solves for those directly. When this flag is **off**, the freespace
places are still repositioned after each loop closure: each place is moved by the
same correction that the optimization applies to the map geometry next to it, so the
places stay lined up with the corrected map. They are simply not solved for as
separate unknowns.

Turn it **on** only if you specifically need the place positions themselves to be
estimated as optimization variables. The cost of turning it on: every place node, the
edges between places, and the links from each place to the mesh all become part of
the batch solve. With a **dense** place layer plus loop closures, eliminating that
graph can produce one very large dense block in the linear algebra (a big "clique"),
which uses enormous memory and can crash the solver. (Concretely, in a long corridor
with a dense L3 it produced tens of GB and a `HessianFactor::Allocate` segfault.)

**Rule of thumb:** don't add a dense derived layer (like freespace places) to the
batch optimization unless you actually need those nodes *estimated*. If you only need
them to stay aligned with the corrected map, leave the flag **off** — they are
repositioned to match the map automatically. (Related: the backend uses METIS
ordering for the solve, which keeps fill-in manageable on loopy graphs; COLAMD on a
long loopy corridor fills in catastrophically.)

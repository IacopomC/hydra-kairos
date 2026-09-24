# Hydra pipeline overview

A navigation map of this repo: the modules, the order data flows through them,
and a one-line description of what each file does. The goal is that you can read
this page and know exactly where to look before opening any source.

Every module below is covered, mirroring the in-source Doxygen on each header.

## Pipeline at a glance

\dot Hydra: how a sensor frame becomes a 3D scene graph
digraph hydra_pipeline {
  rankdir=LR;
  bgcolor="transparent";
  node [shape=box, style="rounded,filled", fillcolor="#eef3fb", fontname="Helvetica", fontsize=10];
  edge [fontname="Helvetica", fontsize=9];

  input [label="input"];
  odom  [label="odometry"];
  aw    [label="active_window\n+ reconstruction"];
  fe    [label="frontend"];
  be    [label="backend"];
  lcd   [label="loop_closure"];
  dsg   [label="3D scene graph", shape=note, fillcolor="#fff7e6"];

  input -> aw;
  odom  -> aw  [label="poses"];
  aw    -> fe  [label="TSDF + mesh"];
  fe    -> be  [label="graph update"];
  fe    -> lcd [label="agents, places"];
  lcd   -> be  [label="loop closures"];
  be    -> dsg [label="optimized"];

  places [label="places (GVD)", shape=box, style="rounded,dashed"];
  rooms  [label="rooms", shape=box, style="rounded,dashed"];
  fe -> places [style=dashed, arrowhead=none, label="builds"];
  be -> rooms  [style=dashed, arrowhead=none, label="builds"];
}
\enddot

The spine is `input → active_window → frontend → backend → scene graph`. Odometry
feeds body poses into the window; loop closures detected from frontend output feed
back into the backend optimization. The `places` (GVD) and `rooms` layers are built
within the frontend and backend respectively, not as standalone stages.

## Modules and data flow

Hydra builds a hierarchical 3D scene graph from sensor input, in roughly this
order:

1. **input**: receive and pack sensor data (images, point clouds, poses) into a
   common per-frame input structure.
2. **odometry**: track the sensor pose frame to frame.
3. **active_window**: keep a moving spatial window of recent data around the
   sensor, and decide what leaves the window for archival.
4. **reconstruction**: fuse input into a volumetric map (TSDF, optional
   semantics, free-space tracking) and extract a surface mesh.
5. **frontend**: turn the local reconstruction into scene-graph primitives
   (mesh, objects, places) for the current window.
6. **places**: build the free-space topology (the places layer / GVD).
7. **rooms**: cluster places into rooms.
8. **backend**: optimize the global scene graph (pose/deformation graph),
   apply loop closures, and run the per-layer update functors.
9. **loop_closure**: detect revisits and propose loop closures to the backend.

Support modules: **common** (shared types, config, module framework),
**utils** (helpers), **gnn** (graph neural net inference), **openset**
(open-vocabulary features), **reconstruction/interpolators** (image sampling).

## File index

### reconstruction
The volumetric map and the integrators that build it from sensor input.
- `voxel_types.h`: the TSDF / semantic / free-space-tracking voxels, their
  spatial-hash blocks, the mesh block, and the per-aspect layer aliases.
- `volumetric_map.h`: the bundle of co-registered voxel layers plus the surface
  mesh, with block allocation/removal, save/load, cloning, and tuple accessors.
- `projective_integrator.h`: fuses a frame of depth and optional semantics into
  the map by projecting voxels into the sensor image and updating distance/weight.
- `marching_cubes.h`: marching-cubes surface extraction for one voxel cube, with
  the per-corner sample type and the triangulation lookup tables.
- `mesh_integrator.h`: builds the surface mesh from the TSDF by running marching
  cubes over dirty blocks in parallel (interiors and shared block faces).
- `mesh_integrator_config.h`: parameters for the mesh integrator (min TSDF weight,
  thread count).
- `semantic_integrator.h`: per-voxel semantic label fusion interface plus a
  maximum-likelihood implementation.
- `projection_interpolators.h`: strategies (nearest / bilinear / adaptive) for
  sampling sensor images at a projected sub-pixel point during integration.
- `tsdf_interpolators.h`: resampling a TSDF layer onto a coarser grid (a
  downsampling implementation behind a factory interface).
- `index_getter.h`: a small thread-safe index dispenser used to parallelize work
  over a fixed list of indices.

### input
Receiving sensor data and packing it into per-frame input for the pipeline.
- `input_module.h`: the pipeline front door; drains the sensor receivers, pairs
  each frame with the body pose, and emits InputPackets on the output queue.
- `data_receiver.h`: base class for a per-sensor input source; queues incoming
  packets and rate-limits them by a minimum timestamp separation.
- `input_packet.h`: one raw input frame (timestamp, body pose, sensor packet),
  convertible to processed InputData.
- `sensor_input_packet.h`: the raw sensor measurement (image / point-cloud
  variants), each knowing how to fill an InputData.
- `input_data.h`: the processed per-frame input the reconstruction stage
  consumes (images, pose, range bounds, features), bound to its sensor.
- `sensor.h`: the Sensor base class (range bounds, extrinsics, point projection,
  view-frustum tests, building alternate range/cloud representations).
- `camera.h`: pinhole RGB-D camera model; projects via intrinsics and builds a
  vertex map and range image from a depth image.
- `lidar.h`: LiDAR model; a spherical projection from angular resolution and FOV
  that builds a range image from a point cloud.
- `sensor_extrinsics.h`: body-to-sensor transform, supplied as identity, explicit
  parameters, or from a Kimera-style calibration file.
- `sensor_utilities.h`: free helpers for view-frustum block tests, finding the
  blocks visible from a sensor pose, and range-image-from-points.
- `input_conversion.h`: helpers that parse an InputPacket into InputData and
  normalize the image data (labels, depth, color, vertex-map frame).

### odometry
Turning the stream of body poses into pose-graph increments for the backend.
- `pose_graph_tracker.h`: the tracker interface plus PoseGraphPacket, the bundle
  of new pose-graph segments and external priors produced per update.
- `pose_graph_from_odom.h`: a tracker that adds one odometry edge between
  successive poses, gated by minimum time and weighted pose-change thresholds.

### active_window
Maintaining a moving spatial window of the volumetric map around the sensor.
- `active_window_module.h`: the base module owning the windowed map and spin
  loop; subclasses fill in how each input packet is fused.
- `active_window_output.h`: the per-pass output packet (body pose, sensor data,
  scene-graph update, archived mesh blocks, current map).
- `reconstruction_module.h`: the active-window subclass that runs TSDF, mesh, and
  robot-footprint integration on each frame.
- `volumetric_window.h`: the window policy (block in-bounds test plus block
  archival), with a fixed-radius spatial implementation.

### frontend
Turning active-window reconstruction output into scene-graph primitives for the
current window: mesh, objects, places, and exploration frontiers.
- `graph_builder.h`: the frontend module; per active-window packet it updates the
  mesh, objects, places, frontiers, and deformation/pose graphs, then merges the
  result into the backend and loop-closure graphs.
- `graph_connector.h`: maintains interlayer parent/child edges, attaching each
  active child to its nearest active parent and rewiring as nodes archive.
- `mesh_segmenter.h`: clusters labeled mesh vertices into object nodes and keeps
  those objects (add/update/merge) current as the mesh grows.
- `surface_places_interface.h`: abstract interface for extracting surface
  (mesh-derived) place nodes and inserting them into the scene graph.
- `place_2d_segmenter.h`: surface-place extractor that clusters labeled mesh
  vertices into 2D places, splits oversized clusters, and maintains overlap edges.
- `place_2d_split_logic.h`: 2D-place geometry: bounding-ellipse fit, recursive
  splitting of oversized places, and the test for whether two places connect.
- `place_mesh_connector.h`: links each active place to its nearest mesh vertices
  and their deformation-graph indices via a KD-tree over a mesh delta.
- `freespace_places_interface.h`: abstract interface for detecting free-space
  place nodes, writing them to the graph, and testing whether a position is free.
- `gvd_place_extractor.h`: free-space place extractor that integrates the TSDF
  into a generalized Voronoi diagram and pulls a sparse, filtered places graph.
- `frontier_extractor.h`: detects exploration frontiers (boundary voxels between
  place-covered free space and the unobserved region) and adds them as nodes.
- `view_database.h`: buffers recent sensor views and assigns each active place a
  semantic feature drawn from the views that can see it.
- `view_selector.h`: a sensor observation paired with a feature vector
  (FeatureView) plus the strategy interface for choosing/fusing a node's feature.

### places
The free-space topology: a Euclidean signed-distance field and generalized
Voronoi diagram of free space, compressed into the sparse places graph.
- `gvd_voxel.h`: per-voxel ESDF/GVD cell state, the block/layer types over it, and
  inline helpers for distance, parent, and Voronoi-membership management.
- `gvd_integrator_config.h`: configuration for the GVD/ESDF integrator and the
  Voronoi membership test, including how parent distinctness is measured.
- `gvd_integrator.h`: incremental integrator turning a TSDF into an ESDF and GVD
  via raise/lower wavefronts, parent tracking, and Voronoi marking.
- `gvd_graph.h`: lightweight graph over Voronoi voxels (position, clearance,
  basis-point count, siblings) with id recycling, feeding the extractors.
- `gvd_parent_tracker.h`: per-voxel bookkeeping of distinct nearest-surface parents
  plus a reference-counted cache of the surface vertices they map to.
- `gvd_merge_policies.h`: pluggable, factory-selectable strategies for choosing the
  representative when two GVD nodes merge.
- `gvd_utilities.h`: free functions for picking the lower candidate distance and
  evaluating the Voronoi condition from parent separation.
- `voxel_templates.h`: local 3x3x3 neighborhood templates (27-bit masks, order
  conversions, rotations) for detecting GVD corner/vertex configurations.
- `graph_extractor_interface.h`: base class for GVD-to-place-graph extractors,
  holding the output layer, the intermediate GVD graph, and the modified/deleted queues.
- `graph_extractor_config.h`: configuration for place-graph extraction (overlap and
  free-space edge parameters, plus compression- and floodfill-specific settings).
- `graph_extractor_utilities.h`: shared edge-proposal helpers (voxel-line
  rasterization, clearance scoring, overlap and free-space edge gathering).
- `compression_graph_extractor.h`: extractor that compresses the GVD onto a coarse
  grid, one node per cell, accumulating edges from underlying GVD adjacencies.
- `floodfill_graph_extractor.h`: extractor that floodfills along Voronoi voxels
  (corners become nodes, intervening voxels become edges) and splits stray edges.
- `nearest_voxel_utilities.h`: spatial queries over a voxel-index set
  (nearest-neighbor and furthest-from-line) used in edge proposal and validation.
- `robot_footprint_integrator.h`: carves the robot's body-frame footprint into the
  map as free space so the body never reads as an obstacle.
- `update_statistics.h`: per-update counters summarizing the integrator's work for
  diagnostics and logging.

### rooms
Clustering the places layer into rooms.
- `graph_filtration.h`: computes the component-count-versus-dilation curve over the
  places layer, tracks per-component lifetimes, and picks a representative threshold.
- `graph_clustering.h`: a cluster-result container plus seeded partitioners (greedy
  modularity and weight-ordered flood fill) that group nodes into communities.
- `room_finder_config.h`: configuration and enums selecting the clustering
  algorithm, threshold-selection strategy, size limits, and logging.
- `room_finder.h`: detects rooms from the places layer (filtration to threshold to
  components to clusters to room layer) and adds room/place, room/archetype, and
  room/room edges.
- `room_utilities.h`: helpers to compute a room's representative position from its
  member places and to wire room-to-room edges from place adjacency.

### backend
Global scene-graph optimization: maintain a pose-graph-and-mesh factor graph,
optimize on loop closures, then run per-layer update functors to keep the graph
consistent with the optimized trajectory and mesh.
- `backend_module.h`: the optimization module: factor-graph maintenance,
  loop-closure-triggered optimization, the update-functor chain, sinks, and status.
- `backend_input.h`: the per-step input bundle (deformation graph, agent updates,
  mesh delta) plus the per-vertex first/last-seen timestamp update.
- `backend_utilities.h`: shared helpers for timestamp lookup, remapping surface-place
  mesh connections through a mesh delta, index merging, and keyframe expansion.
- `pgmo_configs.h`: extends the mesh-deformation config with scene-graph factor
  covariances and robust-solver settings.
- `mst_factors.h`: adds the places layer to the deformation graph via a minimum
  spanning tree plus place-to-mesh edges.
- `merge_proposer.h`: the Merge record, the association-strategy interface, and the
  proposer that pairs candidates and applies a predicate.
- `association_strategies.h`: concrete merge-candidate strategies (pairwise,
  semantic-pairwise, nearest-node, semantic-nearest-node).
- `merge_tracker.h`: applies merge proposals to the shared graph and remembers merge
  history so later proposals can be remapped and merged attributes recomputed.
- `surface_place_utilities.h`: helpers that recompute the surface-place layer after
  mesh optimization (collect, refresh geometry, split, insert nodes and edges).
- `update_functions.h`: the base UpdateFunctor interface plus the per-iteration
  UpdateInfo context and the cleanup/merge hook types.
- `update_objects_functor.h`: tracks active objects, proposes object merges, and
  combines merged-object attributes and mesh connections.
- `update_places_functor.h`: moves places to optimized positions (or interpolates)
  and proposes merges between nearby comparable-size places.
- `update_surface_places_functor.h`: refreshes surface places from the optimized
  mesh, merges co-planar neighbors, and optionally splits oversized places.
- `update_rooms_functor.h`: re-detects rooms from the places layer and rewrites the
  rooms layer.
- `update_buildings_functor.h`: maintains a single building node at the rooms'
  centroid and parents rooms to it.
- `update_frontiers_functor.h`: a no-op update whose cleanup hook removes frontier
  places the robot has reached or that fall inside a real place.
- `update_agents_functor.h`: writes optimized agent poses onto agent nodes and drops
  agents lacking an optimized value.
- `update_archetypes_functor.h`: builds the archetype layer by classifying places
  into motion archetypes from flow metadata, clustering them, and writing one node
  per cluster.
- `zmq_interfaces.h`: a sink publishing the optimized graph over ZeroMQ and an updater
  applying externally received room labels.

### loop_closure
Detecting revisits and proposing loop closures to the backend.
- `lcd_input.h`: the per-update payload listing new agent nodes and freshly archived
  place nodes to consider.
- `loop_closure_module.h`: the threaded module that drains frontend updates, keeps a
  local graph copy, drives the detector per query agent, and pushes deduplicated
  constraints to the backend.
- `loop_closure_config.h`: the top-level config plus the declare_config declarations
  registering the loop-closure config structs.
- `detector.h`: caches per-node descriptors across layers, searches them for a query
  agent, and runs the registration cascade to produce solutions.
- `scene_graph_descriptors.h`: the Descriptor type, the factory interface, and the
  default agent/object/place descriptor factories.
- `gnn_descriptors.h`: graph-network descriptor factories that build a feature tensor
  from a node's subgraph and run it through a learned model.
- `subgraph_extraction.h`: gathers the neighborhood of nodes (fixed radius or grown to
  a node count) that feeds a descriptor.
- `descriptor_matching.h`: descriptor similarity functions (cosine, L1 bag-of-words)
  plus the layer search that ranks candidate matches against a query.
- `registration.h`: the registration-solver interface, a robust point-cloud solver,
  and templated helpers building correspondences between two layers.
- `registration_solution.h`: the value type carrying the relative pose between two
  agent frames, validity, hierarchy level, and optional variance.
- `bow_agent_solver.h`: an agent-layer solver that scores a bag-of-words similarity
  and emits a relative pose from the agents' world poses.

### common
Shared foundation: cross-cutting types, configuration, the module framework,
inter-module queues, and shared scene-graph state.
- `common_types.h`: shared aliases (NodeIdSet, nanosecond TimeStamp) and time-unit
  conversion helpers.
- `dsg_types.h`: imports the scene-graph library types into `hydra` and provides
  node-position accessors for static/dynamic layers and whole graphs.
- `config_utilities.h`: config (de)serialization traits for layer-id-keyed maps and
  quaternions.
- `global_info.h`: pipeline-wide config structs and the GlobalInfo singleton resolving
  robot prefix, logging, label space, color map, sensors, and the active window.
- `label_space_config.h`: describes the label count, color/remap file paths, and the
  dynamic/invalid/object/surface-place/ground label categories.
- `label_remapper.h`: maps fine-grained labels onto coarser ones from a loaded table.
- `semantic_color_map.h`: a two-way label/color lookup with an unknown-color fallback,
  buildable randomly or from a file.
- `robot_prefix_config.h`: derives per-robot node and mesh-vertex symbol prefixes from
  a robot id.
- `message_queue.h`: a mutex-protected FIFO with optional capacity and condition-variable
  waiting, used to pass data between modules.
- `pipeline_queues.h`: the singleton holding the queues that route data between
  frontend, backend, and loop-closure modules.
- `output_sink.h`: an observer template plus its function/method sink implementations
  and group call/instantiate helpers.
- `graph_update.h`: batched node insertion: per-layer pending-node collection and the
  updater that assigns successive symbol ids and applies them to a graph.
- `module.h`: the abstract pipeline-unit interface (start/stop/save plus a description hook).
- `hydra_pipeline.h`: the incremental multi-module coordinator owning shared DSG info
  and module state and driving init/start/stop/save.
- `batch_pipeline.h`: a one-shot pipeline that builds a complete scene graph from a
  finished volumetric map in a single pass.
- `shared_dsg_info.h`: a scene graph plus the mutex, update flag, sequence number,
  layer/prefix maps, and pending merges needed to share it.
- `shared_module_state.h`: bundles the scene-graph views handed to the loop-closure and
  backend modules.
- `launch_callbacks.h`: runs a set of callbacks concurrently (one thread each) and joins them.

### utils
Self-contained support code: data structures, graph and mesh algorithms, I/O,
logging, timing, and formatting.
- `bucket_queue.h`: an approximate priority queue that sorts keys into fixed value
  buckets, caching the lowest non-empty bucket for fast pop.
- `disjoint_set.h`: union-find over node ids, merging by size with a tie-breaker root.
- `id_tracker.h`: a unique-id allocator that recycles freed ids before issuing new ones.
- `active_window_tracker.h`: a filtered layer view over nodes currently active (or
  active when last seen and not yet cleared), keeping just-archived nodes visible one pass.
- `minimum_spanning_tree.h`: computes an MST over a graph layer using inter-node
  Euclidean distance, returning chosen edges, per-node degree, and leaves.
- `nearest_neighbor_utilities.h`: KD-tree nearest-neighbor search over scene-graph nodes
  (optionally grouped by label) and over raw 3D points.
- `mesh_utilities.h`: derives node centroids, object bounding boxes, the active subset of
  a mesh, and an outlier-filtered bounding-box fit from mesh vertices.
- `place_2d_ellipsoid_math.h`: 2D ellipse-overlap geometry (weighted-center curve, shared
  interior point, transverse overlap width).
- `pgmo_mesh_interface.h`: adapters exposing a block-structured mesh layer and a flat mesh
  through the mesh-deformation library's interface.
- `pgmo_mesh_traits.h`: trait overloads letting the mesh-deformation library read/write
  vertices, faces, and per-vertex attributes of a scene-graph mesh.
- `pgmo_glog_sink.h`: forwards mesh-deformation-library log entries into glog.
- `layer_io.h`: binary serialization of voxel layers, with per-type specializations and
  type-tag validation.
- `csv_reader.h`: loads a delimited file into memory and exposes rows by index and
  columns by header name.
- `log_utilities.h`: owns the log directory tree, resolves per-namespace and per-timer
  paths, and runs exit callbacks at destruction.
- `timing_utilities.h`: wall-clock timing via a singleton recorder plus a scoped timer.
- `display_utilities.h`: string formatting for byte counts, dynamic vectors, and node ids.
- `printing.h`: Eigen formatting helpers (index display and a shared bracketed IO format).

### gnn
Graph-neural-net inference, wrapping the ONNX Runtime inference library.
- `tensor.h`: a dense, reference-counted, typed multi-dimensional buffer for inference
  I/O, with an element-type enum and typed/Eigen-map accessors.
- `ort_utilities.h`: field info describing a model input/output, tensor validation and
  conversion to runtime values, tensor allocation, and session introspection.
- `gnn_interface.h`: loads a model and runs inference, mapping named input tensors to
  named output tensors (including dynamic output-size resolution).

### openset
Open-vocabulary (open-set) semantic feature embeddings.
- `openset_types.h`: the core embedding type aliases (a single vector, a matrix of
  embeddings, and a keyed map of embeddings).
- `embedding_distances.h`: the metric interface (paired distance/score) plus cosine, L1,
  L2, and softmax-style score implementations.
- `embedding_group.h`: a named collection of embeddings that scores entries against a
  query under a chosen metric and returns the best match.
- `embedding_pooling.h`: the pooling interface plus mean, weighted-mean, and max
  strategies that collapse many embeddings into one.

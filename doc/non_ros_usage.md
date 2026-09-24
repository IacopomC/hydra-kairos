# Running Hydra without ROS (via the Python bindings)

This doc describes how to run `hydra-kairos` on pre-recorded RGB-D data as a
plain Python process: frames, poses and intrinsics are read from files on disk.
The Python pipeline can also stream the graph live to RViz over ZMQ.

All file paths below are relative to the `hydra-kairos/` root.

---

## 1. Context: ROS vs. non-ROS

The project is split across two repos:

| Repo | Purpose | Needs ROS? | Rosbags? | tf2? |
| --- | --- | --- | --- | --- |
| `hydra-kairos/` (this one) | C++ pipeline + Python bindings | No | No | No |
| `hydra-kairos-ros/` (separate) | ROS wrappers for live robots | Yes | Yes | Yes |

To test the reconstruction / scene-graph pipeline on a dataset, stay inside
`hydra-kairos/`: the pipeline takes its inputs from a scene folder (§3) or from
your own loader (§7).

The visualizer (`hydra_visualizer`) lives in `hydra-kairos-ros/` and builds in a
catkin workspace, so visualization requires sourcing
`<catkin_ws>/devel/setup.bash`.

---

## Quickstart — TBD

End-to-end recipe for the TBD pedestrian dataset (CaBot + ZED2). It follows four
steps: convert the source data into the `FileDataLoader` layout (§3), generate
semantic labels (see §5 caveat), run the pipeline, visualize. The `tbd` dataset
config belongs to KAIROS, so step 3 runs through the `kairos` CLI, which wraps
this pipeline and adds the flow layer.

Source the Python environment and the catkin workspace in each terminal:

```bash
source <python_venv>/bin/activate
source <catkin_ws>/devel/setup.bash
```

Replace `<kairos_ws>` and `<tbd_root>` with your local paths.

```bash
# 1. Convert: TBD data3d folder + cartographer trajectory → FileDataLoader layout
python3 <kairos_ws>/src/hydra-kairos/scripts/tbd_to_filedataloader.py \
    --data3d <tbd_root>/Robot_data/2022-12/cabot_2022-12-07-12-15-36_data3d \
    --trajectory <tbd_root>/Robot_data/2022-12/cabot_2022-12-07-12-15-36-trajectory.yaml \
    --output ~/Downloads/scene_tbd_2022-12-07-12-15-36 \
    --max-frames 500

# 2. Generate ade20k_outdoor labels (replaces the blank labels_*.png the
#    converter wrote)
python <kairos_ws>/src/hydra-kairos/scripts/generate_labels.py \
    ~/Downloads/scene_tbd_2022-12-07-12-15-36 --overwrite

# 3. Terminal 1 — run the pipeline with the TBD config (in the kairos repo)
kairos run ~/Downloads/scene_tbd_2022-12-07-12-15-36 \
    -c tbd -l ade20k_outdoor \
    -m 100 \
    -o ~/Downloads/hydra_out_tbd --no-publish --no-enable-lcd

# 4. Terminal 2 — visualize from the saved DSG
roslaunch hydra_visualizer hydra_visualizer.launch \
    scene_graph:=$HOME/Downloads/hydra_out_tbd/scene_tbd_2022-12-07-12-15-36/backend/dsg_with_mesh.json \
    color_mesh_by_label:=false
```

### Notes

- **`generate_labels.py` fills every missing or blank `labels_*.png` and keeps
  labels that already have content**; `--overwrite` regenerates all of them.
  The converter keeps labels already present in the output folder, so
  re-converting a labelled scene leaves its labels in place.
- **`color_mesh_by_label:=false`** displays the RGB-textured mesh. The default (`true`, set in
  `hydra_visualizer.launch`) recolours the mesh by semantic label.
  `hydra visualize` forwards extra args to roslaunch, so the same flag works
  there.
- **Hydra alone.** `python -m hydra_python run <scene> -c <config> -l <labelspace>`
  builds the scene graph with Hydra only (no flow layer), using one of this
  repo's dataset configs (`config/datasets/`).

---

## 2. The offline entry point

### CLI

```bash
python -m hydra_python run <scene_path> \
    -c habitat \
    -l ade20k_mp3d \
    --no-publish --no-progress \
    -m 50
```

Useful flags:

- `-c / --config-name` — which config bundle to load (`habitat` is the standard
  one).
- `-l / --labelspace` — semantic label space (e.g. `ade20k_mp3d`).
- `-o / --output-path` — where to dump the produced scene graph and logs.
  **Required for offline use**: `pipeline.save()` writes the scene graph here,
  and that file is what you visualize or reload.
- `-m / --max-steps` — cap the number of frames (good for smoke tests).
- `--openset-model` — load a CLIP-based open-vocabulary model that populates
  the place features (see §5).
- `--publish / --no-publish` — ZMQ visualization publisher. Leave on to watch
  the graph build up live in RViz (see §10.3); turn off for headless batch
  jobs.

### What the CLI actually does

`python/src/hydra_python/commands/run.py` performs, in order:

1. `data = hydra_python.get_dataloader(scene_path)` — picks a loader based on
   the path (see §3 below).
2. `pipeline = load_pipeline(data, config_name, labelspace, ...)` — builds a
   `HydraPipeline` and hands it the camera/sensor from `data.sensor`.
3. `DataLoader.run(pipeline, data, max_steps=...)` — iterates frame by frame
   and calls `pipeline.step(...)` on each packet.

The Python bindings are the offline API: the same three calls work from your
own script or notebook.

---

## 3. Folder-based data: `FileDataLoader`

`get_dataloader()` dispatches
based on the argument:

- A **directory** → `FileDataLoader(scene_path)` (folder-based, described
  here).
- A `.glb` file → `HabitatInterface(scene_path)` (Habitat-rendered).

### Expected folder layout

```
<scene>/
  poses.csv           # world_T_body trajectory, one row per frame
  camera_info.yaml    # fx, fy, cx, cy, width, height
  rgb_0000000.png     # uint8 HxWx3
  rgb_0000001.png
  ...
  depth_0000000.tiff  # uint16 (mm) or float32 (m)
  depth_0000001.tiff
  ...
  labels_0000000.png  # integer semantic labels, HxW
  labels_0000001.png
  ...
```

The index is a zero-padded 7-digit frame number aligned with the order of rows
in `poses.csv`.

### `poses.csv` format

One header row, then rows of:

```
timestamp_ns, px, py, pz, qw, qx, qy, qz
```

- `timestamp` is an integer (nanoseconds).
- Translation in meters.
- Quaternion order is **wxyz**; scipy returns xyzw, so reorder when
  exporting.
- The pose is `world_T_body` with the body at the camera's optical frame
  (x right, y down, z forward): the loader uses identity camera extrinsics, so
  compose any body↔camera offset into the pose before writing it
  (`data_generation.md` §3.4).

### `camera_info.yaml` format

```yaml
fx: 525.0
fy: 525.0
cx: 319.5
cy: 239.5
width: 640
height: 480
```

These six pinhole parameters are the whole file.

---

## 4. What every frame looks like: `InputPacket`

Each frame is turned into an `InputPacket` that
the pipeline consumes:

| Field | Type | Notes |
| --- | --- | --- |
| `timestamp` | `int` | nanoseconds |
| `world_q_body` | `np.ndarray (4,)` | quaternion in **wxyz** order |
| `world_t_body` | `np.ndarray (3,)` | translation in meters |
| `color` | `np.ndarray` | `uint8` HxWx3, or `float32` in [0,1] |
| `depth` | `np.ndarray` | `uint16` (mm, auto-converted to meters) or `float32` (meters) |
| `labels` | `np.ndarray` | integer semantic labels, HxW |
| `extras` | `Dict[str, Any]` | free-form side-channel passed through to `pipeline.step(**extras)` |

---

## 5. Required inputs

### Required

- **RGB** (or at least something you can pass as `color`).
- **Depth** aligned to the color image.
- **Per-frame pose** in world frame.
- **Camera intrinsics** — six scalars: `fx, fy, cx, cy, width, height`.

### The one caveat: semantic labels

`FileDataLoader.next()` unconditionally reads `labels_<idx>.png`. If your
dataset has no segmentation, you have three options:

1. **Pre-run a segmenter** and dump labels to disk. The
   `--openset-model` CLI flag wires CLIP into the pipeline to populate the
   place features; the per-pixel labels still come from disk.
2. **Subclass `FileDataLoader`** and override `next()` to return zeros for
   `labels`:
   ```python
   class FileDataLoaderNoLabels(FileDataLoader):
       def next(self):
           packet = super().next()
           if packet is None:
               return None
           packet.labels = np.zeros(packet.depth.shape, dtype=np.int32)
           return packet
   ```
3. **Write a fresh loader** (see §7).

---

## 6. Intrinsics without a YAML file (programmatic path)

To set the intrinsics in code, build the `Camera` directly:

```python
from hydra_python._hydra_bindings import Camera, CameraConfig, ExtrinsicsConfig

config = CameraConfig()
config.min_range = 0.1
config.max_range = 5.0
config.width  = 640
config.height = 480
config.fx = 525.0
config.fy = 525.0
config.cx = 319.5
config.cy = 239.5
config.extrinsics = ExtrinsicsConfig()   # identity body→camera
sensor = Camera(config, "camera")
```

Pass that `sensor` into `HydraPipeline.from_config(yaml_config, sensor, ...)`.

---

## 7. Custom dataloader skeleton

For a different on-disk layout, for labels generated in code, or to inject
custom fields via `extras`, implement the abstract
`DataLoader`:

```python
import hydra_python
from hydra_python.data_loader import DataLoader, InputPacket
from hydra_python._hydra_bindings import Camera, CameraConfig, ExtrinsicsConfig
import numpy as np


class MyLoader(DataLoader):
    def __init__(self, root, intrinsics):
        self._root = root
        self._frames = sorted(root.glob("frame_*"))
        self._intrinsics = intrinsics
        self._idx = 0

    @property
    def sensor(self):
        cfg = CameraConfig()
        for k, v in self._intrinsics.items():
            setattr(cfg, k, v)
        cfg.extrinsics = ExtrinsicsConfig()
        return Camera(cfg, "camera")

    def __len__(self):
        return len(self._frames)

    def reset(self):
        self._idx = 0

    def next(self):
        if self._idx >= len(self._frames):
            return None
        frame_dir = self._frames[self._idx]
        self._idx += 1
        # ... load color, depth, labels, pose from frame_dir ...
        return InputPacket(
            timestamp=ts_ns,
            world_q_body=q_wxyz,
            world_t_body=t_xyz,
            color=color,
            depth=depth,
            labels=labels,
            extras={},  # anything extra the pipeline should see
        )


DataLoader.register(MyLoader)
```

Then in your driver script:

```python
data = MyLoader(root, intrinsics)
pipeline = hydra_python.load_pipeline(data, "habitat", "ade20k_mp3d")
hydra_python.DataLoader.run(pipeline, data, max_steps=500)
```

---

## 8. Smoke test

After converting one scene, verify the pipeline runs end to end:

```bash
python -m hydra_python run /path/to/converted/scene \
    -c habitat -l ade20k_mp3d \
    --no-publish --no-progress \
    -m 50 \
    -o /tmp/hydra_smoke
```

What to check:

- No exceptions, 50 frames processed, progress-bar counter monotonic.
- `/tmp/hydra_smoke/<scene_name>/` contains the logs and the serialized scene
  graph.
- Load the scene graph with `spark_dsg` (from `spark-dsg-kairos`, e.g.
  `spark_dsg.DynamicSceneGraph.load(...)`) and confirm it has non-trivial
  nodes/edges.

If it crashes:

- `ValueError: # of times != # of poses` → `poses.csv` malformed.
- Segfault on first frame → check the intrinsics: width/height must match the
  image shape.
- Pipeline runs but output graph is empty → check depth units (meters vs mm)
  and quaternion order (wxyz vs xyzw).

---

## 9. Key files (index)

| File | What lives here |
| --- | --- |
| `python/src/hydra_python/__init__.py` | Re-exports everything from submodules. |
| `python/src/hydra_python/__main__.py` | CLI entry (`python -m hydra_python ...`). |
| `python/src/hydra_python/commands/run.py` | `run` subcommand — the offline entry point. |
| `python/src/hydra_python/commands/main.py` | Click command group wiring. |
| `python/src/hydra_python/data_loader.py` | `InputPacket`, `DataLoader`, `FileDataLoader`, `get_dataloader`. |
| `python/src/hydra_python/trajectory.py` | `Trajectory.from_csv`, pose parsing. |
| `python/src/hydra_python/pipeline.py` | `HydraPipeline` wrapper, `load_pipeline`. |
| `python/src/hydra_python/simulators/habitat.py` | Example of programmatic sensor setup + a simulator-backed loader. |
| `python/tests/test_image_bindings.py` | Worked examples of accepted image dtypes/shapes. |
| `python/tests/test_sensor_input.py` | Worked examples of feeding a single frame into the pipeline. |

---

## 10. Viewing the result

After a successful run with `-o <out>`, the scene graph is written to:

```
<out>/<scene_stem>/
    backend/
        dsg.json              # nodes + edges only
        dsg_with_mesh.json    # same, plus the reconstruction mesh — use this
    logs/                     # glog files
    config.yaml               # resolved Hydra config used for the run
```

Use `dsg_with_mesh.json` for visualization.

### 10.1 RViz from a saved file (recommended)

The `hydra_visualizer` package in the sibling `hydra-kairos-ros` workspace has a
`GraphFromFile` mode that reads a saved DSG JSON directly:

```bash
source /path/to/catkin_ws/devel/setup.bash

roslaunch hydra_visualizer hydra_visualizer.launch \
    scene_graph:=$HOME/hydra_out/scene_0001/backend/dsg_with_mesh.json
```

The fixed frame defaults to `map` (see `launch/hydra_visualizer.launch` for the arg list).

### 10.2 Open3D from Python (no ROS)

`spark_dsg[viz]` ships a standalone Open3D renderer that runs from plain Python:

```python
import spark_dsg as dsg
from spark_dsg.open3d_visualization import render_to_open3d

G = dsg.DynamicSceneGraph.load("/path/to/dsg_with_mesh.json")
render_to_open3d(G)
```

It publishes on ZMQ port `tcp://127.0.0.1:8001` — don't run it at the same
time as a streaming `hydra run` using the same port.

### 10.3 Live streaming view during a run

To watch the graph build up in real time while the pipeline is running, leave
`--publish` on (the default) and launch the streaming visualizer in a separate
terminal:

```bash
hydra visualize
```

It opens RViz with the ZMQ subscriber pointed at port 8001. Start it before
`hydra run`, or at least before the first frames are published — frames
emitted before a subscriber connects are lost.

### 10.4 Troubleshooting

- `<out>/<scene_stem>/backend/` missing or empty → the run didn't reach
  `pipeline.save()`. Check stderr for exceptions, and make sure you passed `-o`.
- RViz opens but nothing renders → set the fixed frame to `map`, then verify
  the graph has nodes with the Python counts below.
- Quick counts check:
  ```python
  from spark_dsg import DynamicSceneGraph
  G = DynamicSceneGraph.load("/path/to/dsg_with_mesh.json")
  for lid in G.layer_ids:
      L = G.get_layer(lid)
      print(f"layer {lid}: {L.num_nodes()} nodes, {L.num_edges()} edges")
  ```
  If every layer is zero, the reconstruction received unusable input (depth,
  poses, intrinsics). The common causes are listed in §8 and in
  `doc/data_generation.md` §7 (common gotchas).

---

## 11. Offline loop closure with BoW

With the default flags, the offline pipeline runs LCD at the **objects** and
**places** layers via TEASER (see `doc/lcd/params.md §9`). For datasets with
sparse semantics or repetitive geometry (TBD, single halls), this can
either fail to fire on real revisits (no objects) or fire too eagerly on
look-alike places. Adding BoW input at the agent layer pre-filters
candidates by visual similarity before the geometric solve runs.

### 11.1 One-time setup

```bash
# Download an ORB vocabulary (~50 MB, once per machine).
bash <kairos_ws>/src/hydra-kairos/scripts/fetch_orb_vocab.sh
# default output: ~/.cache/hydra-kairos/orb_vocab.txt
```

The vocabulary is ORB_SLAM2's `ORBvoc.txt`.

### 11.2 Running with BoW enabled

Add `--bow-vocab` to the standard `hydra run` command:

```bash
python -m hydra_python run <scene> \
    -c <config> -l ade20k_outdoor \
    -m 1500 \
    -o ~/Downloads/hydra_out \
    --bow-vocab ~/.cache/hydra-kairos/orb_vocab.txt
```

Other producer flags:

- `--bow-features N` — number of ORB features per frame. Default `1000`.
- `--bow-color-is-rgb / --bow-color-is-bgr` — channel order of
  `packet.color`. Default `--bow-color-is-rgb`, which matches
  `FileDataLoader`. Pass
  `--bow-color-is-bgr` for a custom dataloader that returns BGR.

`--bow-vocab` requires `--enable-lcd` (the default).

### 11.3 What to look for in the logs

Three lines confirm BoW is wired correctly:

1. **Vocab loaded**: `[BowProducer] loaded DBoW3 vocab from … (size=N, k=10, L=6)`
   — printed once at startup. If absent, the producer was not constructed;
   check the `--bow-vocab` path (see §11.5).
2. **Producer pushing**: `[bow_producer] pushed BoW for pose_id=N (total=…)`
   — printed at first 5 emissions and every 50th after. If this never
   appears, no keyframes have been accepted by the Python `KeyframeTracker`
   (e.g. trajectory-only motion below `min_pose_separation`).
3. **Frontend assigning**: `[Hydra Frontend] assigned N bow vectors of M original`
   (VLOG(3)). `M` is
   the cumulative cached message count; in steady state `M - N` stays
   small and bounded. If `M` grows monotonically without `N` keeping up,
   pose_ids are misaligned — see §11.5.

With those three lines present, **agent-layer LCD detect calls report
non-zero valid matches** on revisits. With BoW disabled, every agent-layer
detect call reports 0 valid matches.

### 11.4 What the producer does (architecture)

```
   FileDataLoader               KeyframeTracker
       packet                    (Python)
       │                            │
       │  (timestamp, world_T_body, │  mirrors PoseGraphFromOdom::update
       │   color)                   │
       ▼                            ▼
  data_callback ── BowProducerCallback ── for each accepted (pose_id, color):
                                          │
                                          ▼
                                BowProducer.compute_and_push(rgb, robot_id, pose_id)
                                          │  (C++ binding,
                                          │   python/bindings/src/python_bow.cpp)
                                          ▼
                                cv::ORB::detectAndCompute
                                          │
                                          ▼
                                DBoW3::Vocabulary::transform
                                          │
                                          ▼
                                PipelineQueues::bow_queue.push(BowQuery)
                                          │
                                          ▼
                                GraphBuilder::assignBowVectors
                                  matches BowQuery.pose_id against the
                                  agent_key_map_ entry
                                          │
                                          ▼
                                AgentNodeAttributes.dbow_ids /
                                AgentNodeAttributes.dbow_values populated
```

The Python `KeyframeTracker` is critical: the C++ tracker filters input
frames by `min_pose_separation` (default 0.5 m), `min_time_separation_s`
(default 0.2 s), so the agent key (`node.key`) counts *accepted*
keyframes. The Python mirror has to apply
the same rule so the `pose_id` it stamps on each `BowQuery` matches the
agent key the C++ side will assign.

If you tune the C++ tracker via `frontend.pose_graph_tracker:` in your
dataset YAML, also pass the matching values to the producer via the
keyword args of `BowProducerCallback`. These are set in code; the CLI has
no flags for them.

### 11.5 Troubleshooting

- **No `[BowProducer] loaded`** → `--bow-vocab` path is wrong, or you
  rebuilt the bindings without DBoW3 (check that
  `python/CMakeLists.txt` still has the `FetchContent_Declare(dbow3 ...)`
  block and that `pip install -e .` ran without
  `Compatibility with CMake < 3.5 has been removed` — that error means
  `CMAKE_POLICY_VERSION_MINIMUM=3.5` isn't set; see the comment in
  `python/CMakeLists.txt`).
- **`bow_queue not allocated; …`** thrown from `BowProducer.compute_and_push`
  → `frontend.lcd_use_bow_vectors` is false in the merged config. The
  default is true; check that nothing in your dataset YAML or LCD YAML
  override flipped it.
- **`assigned N bow vectors of M original` with M unbounded** → the
  Python `KeyframeTracker` is out of sync with `PoseGraphFromOdom::update`.
  Most common cause: the dataset YAML overrides `frontend.pose_graph_tracker`
  but the Python mirror is using the defaults (0.5 m / 0.2 s / 0.1).
  Either drop the YAML override or pass matching values in code.
- **Agent-LCD still reports 0 valid matches** but BoW lines look fine →
  `lcd.agent.search.min_score` is too tight for your descriptors. Loosen
  `min_score` and `min_score_ratio` one rung at a time.

### 11.6 Key files

| File | What lives here |
| --- | --- |
| `python/bindings/src/python_bow.cpp` | C++ `BowProducer` (ORB + DBoW3 + `bow_queue.push`). |
| `python/bindings/include/hydra/bindings/python_bow.h` | Header. |
| `python/src/hydra_python/bow_producer.py` | `KeyframeTracker` (mirror) + `BowProducerCallback` (data_callback). |
| `python/CMakeLists.txt` | DBoW3 FetchContent + opencv_features2d link. |
| `scripts/fetch_orb_vocab.sh` | Idempotent vocab download. |
| `python/src/hydra_python/commands/run.py` | `--bow-vocab` / `--bow-features` / `--bow-color-is-rgb` CLI flags. |

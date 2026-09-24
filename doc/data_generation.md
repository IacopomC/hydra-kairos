# Generating data for Hydra's `FileDataLoader`

Practical recipe for producing the on-disk layout Hydra's offline pipeline
expects, starting from an RGB-D dataset with poses. For the exact loader contract
and entry-point details see `non_ros_usage.md`.

---

## 1. Target layout

Every scene is a flat directory with one row per frame:

```
scene_0001/
    camera_info.yaml
    poses.csv
    rgb_0000000.png
    rgb_0000001.png
    ...
    depth_0000000.tiff
    depth_0000001.tiff
    ...
    labels_0000000.png
    labels_0000001.png
    ...
```

Frame indices are zero-padded 7-digit integers, aligned with the order of rows
in `poses.csv`. Every index must have all three images (rgb/depth/labels)
present, otherwise `FileDataLoader.next()` fails.

---

## 2. The five streams you need to produce

| File | Type / dtype | Per-frame or global | Notes |
| --- | --- | --- | --- |
| `rgb_<i>.png` | uint8 HxWx3 | per frame | Colour image. |
| `depth_<i>.tiff` | uint16 (mm) **or** float32 (m) | per frame | Metric depth, aligned to `rgb_<i>.png`. |
| `labels_<i>.png` | integer HxW | per frame | Per-pixel semantic labels. Zeros ok. |
| `poses.csv` row | — | per frame | `timestamp_ns, px, py, pz, qw, qx, qy, qz`. |
| `camera_info.yaml` | — | global | `fx, fy, cx, cy, width, height`. Applied to every frame. |

---

## 3. Where each field comes from

### 3.1 RGB

Any HxWx3 uint8 image works. If your source is grayscale, upcast to 3
channels. Channel order: BGR is fine (OpenCV native).

### 3.2 Depth

Three common situations:

- **Native RGB-D** (Kinect, Xtion, ZED, RealSense, iPhone LiDAR): extract the
  already-registered depth topic / stream directly.
- **Stereo pair**: rectify first, compute disparity, convert to depth via
  `Z = fx · baseline / disparity`. OpenCV `StereoSGBM` is the usual baseline;
  HITNet / RAFT-Stereo / CREStereo are pretrained upgrades for textureless
  indoor scenes.
- **Monocular with no metric scale**: use a metric monocular depth model
  (Metric3D, ZoeDepth, Marigold); scale error compounds into Hydra's place
  and room detection.

Critical requirement regardless of source: depth must be **aligned** to the
colour image — same intrinsics, same pose, same pixel grid. A ROS depth topic
registered to the RGB stream satisfies this; raw depth from a
separately-mounted sensor must be registered to the colour camera first.

**Units**: Hydra's loader auto-detects by dtype. uint16 is interpreted as
millimetres, float32 as metres. Do not save float depth in millimetres — it
will be read as metres and everything will be 1000× too far.

**Encoding**: save as TIFF. Most PNG writers are 8-bit and silently quantise
depth. Use `cv2.imwrite("depth.tiff", uint16_array)`.

### 3.3 Labels

`FileDataLoader.next()` reads `labels_<i>.png` unconditionally. Three options,
in order of effort:

1. **Zeros.** Write an all-zeros `uint16` HxW file for every frame. The
   pipeline runs; flow-on-place-node and room-category aggregation may
   degrade. Fine for pilot runs and smoke tests.
2. **Pre-segment.** Run a semantic segmenter (Mask2Former, SegFormer, etc.)
   into the label IDs of your chosen `--labelspace` (e.g. `ade20k_mp3d` has
   40 classes; see `python/src/hydra_python/semantics/` for the mapping).
3. **Open-set via CLIP.** Pass `--openset-model ViT-L/14` at run time. This
   populates place features from CLIP. The loader still reads the labels
   file, and zeros work for the on-disk part.

### 3.4 Poses

This is the most error-prone field — most "my DSG is empty" bugs end up
being pose bugs. Target format in `poses.csv`:

```
timestamp_ns, px, py, pz, qw, qx, qy, qz
```

- Integer nanoseconds (`int(t_sec * 1e9)`).
- Translation in metres.
- Quaternion in **wxyz** order. scipy and most Python rotation libraries
  return xyzw — **convert before writing**.
- The pose must be `world_T_camera_optical`, i.e. the pose of the camera's
  optical frame expressed in world coordinates.

Three common sources, with the transform you need to apply:

**(a) Already `world_T_camera_optical`** (e.g. ScanNet, Replica, some SLAM
outputs): use as-is. Just reorder the quaternion if it's xyzw.

**(b) Body / base-link pose + static camera mount** (most ROS datasets): compose
with the static body→camera extrinsic (published on `/tf_static` or derivable
from the URDF), plus the REP-103 rotation (see 3.4.1).

**(c) 2D planar odometry** (AMCL output, wheel odometry on ground robots):
lift to 3D as yaw about z. With planar `(x, y, θ)`:

```python
q_wxyz = np.array([np.cos(θ/2), 0.0, 0.0, np.sin(θ/2)])
t_world_base = np.array([x, y, 0.0])
```

Then compose with the body→camera extrinsic as in (b).

#### 3.4.1 REP-103: the rotation everyone forgets

ROS defines two different frame conventions:

- **Base-link / body frame**: x forward, y left, z up.
- **Camera optical frame**: x right, y down, z forward.

The rotation from base to optical is a fixed 120° rotation. As a quaternion
it's:

```python
Q_BASE_TO_OPTICAL_XYZW = np.array([-0.5, 0.5, -0.5, 0.5])
```

Many datasets publish a static `/tf` from base to camera with identity
rotation (only a translation), leaving the optical rotation implicit in the
projection matrix. If you compose without applying the REP-103 rotation,
Hydra will think the camera is pointing along base-link's z-axis — straight
up — and the reconstruction will be a vertical sliver of the
ceiling. Symptom: empty places layer, empty rooms, a few spurious nodes near
the start pose.

Worked composition:

```python
from scipy.spatial.transform import Rotation as R

# world_T_base from your source data
R_world_base = R.from_euler("z", θ)                 # planar yaw
t_world_base = np.array([x, y, 0.0])

# base_T_camera_optical: translation from static TF, rotation = REP-103
t_base_optical = np.array([0.2, 0.0, 1.26])         # dataset-specific mount
R_base_optical = R.from_quat([-0.5, 0.5, -0.5, 0.5])

# world_T_camera_optical
t_world_optical = R_world_base.apply(t_base_optical) + t_world_base
R_world_optical = R_world_base * R_base_optical

q_xyzw = R_world_optical.as_quat()
q_wxyz = np.concatenate([[q_xyzw[3]], q_xyzw[:3]])  # reorder → wxyz
```

### 3.5 Camera intrinsics

Hydra uses a pinhole model with six scalars:

```yaml
fx: 525.0
fy: 525.0
cx: 319.5
cy: 239.5
width: 640
height: 480
```

If your source is a ROS `sensor_msgs/CameraInfo`, pull from the rectified
`P` matrix, not the raw `K` matrix:

```python
fx = P[0,0]
fy = P[1,1]
cx = P[0,2]
cy = P[1,2]
```

If you're starting from raw imagery, rectify first and use the
post-rectification intrinsics.

---

## 4. Temporal alignment

RGB, depth, and poses rarely carry the same timestamps. For 5–10 Hz data a
50 ms nearest-neighbour tolerance works well:

1. Pick the image stream as the reference (it's usually the slowest).
2. For each image timestamp, find the nearest depth and nearest pose within
   tolerance.
3. Drop the frame if either is missing.

Always log how many frames you drop. More than a few percent usually means
the tolerance is too tight, the streams have clock drift, or the pose file
covers a different session than the image bag.

If the image timestamps themselves are bursty (left and right cameras of a
stereo pair publishing separately), match left↔right by timestamp first with
a small buffer, then match the pair's average timestamp to poses.

---

## 5. Reference implementation

`scripts/tbd_to_filedataloader.py` is a worked example targeting one scene of
the TBD pedestrian dataset (CaBot robot, ZED2). Input: the scene's `data3d`
folder (sparse RGB frames, dense depth in mm, a timestamp table) and its
cartographer trajectory. Output: a full `FileDataLoader` scene directory.

What it implements end-to-end:

- Iterating the sparse RGB frames and looking up, for each, the matching depth
  frame and the nearest trajectory pose (`load_timestamps`,
  `nearest_pose_idx`).
- Parsing the cartographer trajectory (`load_trajectory`).
- Composing the `base_link`-in-`map` pose with the static body→camera mount
  and the optical-frame convention to produce `world_T_camera_optical`
  (`compose_world_optical`).
- Writing zeros for labels, plus the `poses.csv` / `camera_info.yaml` headers
  and per-frame images.

For a new dataset, the parts most likely to need re-writing are the trajectory
parser, the frame/timestamp matching, and the static body→camera extrinsic.
Everything else (the `FileDataLoader` output layout, quaternion ordering, unit
conventions) stays the same across datasets.

---

## 6. Smoke test

Before batching a whole dataset, convert a small window and run Hydra with
`-m 50`:

```bash
python -m hydra_python run scene_0001 \
    -c habitat -l ade20k_mp3d \
    -m 50 \
    -o ~/hydra_smoke
```

Then check the node count of each output layer:

```python
from spark_dsg import DynamicSceneGraph
G = DynamicSceneGraph.load("~/hydra_smoke/scene_0001/backend/dsg_with_mesh.json")
for lid in G.layer_ids:
    print(f"layer {lid}: {G.get_layer(lid).num_nodes()} nodes")
```

If the places layer has non-zero nodes, proceed to the full run and to
visualization (`non_ros_usage.md` §10).

---

## 7. Common gotchas

- **Depth saved as PNG instead of TIFF.** Most PNG writers are 8-bit — you
  lose all range past ~25 cm. Use TIFF for uint16.
- **Depth in float32 millimetres.** Hydra reads float32 as metres. Your scene
  ends up 1000× too far.
- **REP-103 skipped.** Camera orientation is 90° off; reconstruction is
  vertical mush. Symptom: near-empty places layer.
- **Quaternion xyzw instead of wxyz.** Orientations scramble. Symptom:
  reconstruction fragments as the robot moves.
- **Rectified images with raw intrinsics, or vice-versa.** Depth projections
  land in the wrong pixels; reconstruction has doubled or missing geometry.
- **Depth from one camera, RGB from another.** Unregistered depth and colour
  with different poses/intrinsics. Symptom: places layer empty despite clean
  depth maps.
- **Timestamps as floats in `poses.csv`.** The loader expects integer
  nanoseconds. Use `int(t_sec * 1e9)`.
- **No `-o` flag on `hydra run`.** Pipeline runs, but `pipeline.save()`
  doesn't persist anywhere findable. See `non_ros_usage.md` §2.
